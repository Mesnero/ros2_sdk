import socket
import threading
import msgpack
from typing import List, Dict, Any, Optional
from rx.subject import Subject
from dataclasses import dataclass, field
import zmq
import platform

@dataclass
class TrajPoint:
    """
    A trajectory point used for send_trajectory.
    The accelerations field is optional; if not provided, it defaults to an empty list.
    """
    positions: List[float]
    velocities: List[float]
    effort: List[float]
    seconds: int
    nanoseconds: int
    accelerations: List[float] = field(default_factory=list)

class ROS2SDK:
    def __init__(self):
        # ZeroMQ specific attributes.
        self._zmq_context: Optional[zmq.Context] = None
        self._zmq_socket_recv: Optional[zmq.Socket] = None
        self._zmq_socket_send: Optional[zmq.Socket] = None        
        self._protocol: Optional[str] = None  # "TCP", "UDS", 0MQ
        
        self._endpoint_recv: Optional[str] = None
        self._endpoint_send: Optional[str] = None

        self._recv_thread: Optional[threading.Thread] = None
        self._running: bool = False

        # Subjects for reactive streams.
        self._feedback_subject: Subject = Subject()
        self._state_subject: Subject = Subject()

    def connect(self, protocol: str, params: Dict[str, Any]):
        """
        Connects to the communication channel.
        For TCP, params should include: 'ip', 'port_recv', 'port_send'
        For UDS, params should include: 'path_recv', 'path_send'
        For 0MQ, params should include: 'endpoint_recv', 'endpoint_send'
        """
        self._protocol = protocol.upper()
        self._zmq_context = zmq.Context()

        if self._protocol == "TCP":
            ip = params.get("ip")
            port_recv = params.get("port_recv")
            port_send = params.get("port_send")
            if ip is None:
                raise ValueError("For TCP, 'ip' must be provided in params.")
            if port_recv is None or port_send is None:
                raise ValueError("For TCP, 'port_recv' and 'port_send' must be provided in params.")
            self._endpoint_recv = f"tcp://{ip}:{port_recv}"
            self._endpoint_send = f"tcp://{ip}:{port_send}"
                        
        elif self._protocol == "UDS":
            if platform.system() == "Windows":
                raise ValueError("UDS is not supported on Windows.")
            path_recv = params.get("path_recv")
            path_send = params.get("path_send")
            if path_recv is None or path_send is None:
                raise ValueError("For UDS, 'path' must be provided in params.")
            self._endpoint_recv = "ipc://" + path_recv
            self._endpoint_send = "ipc://" + path_send
            
        elif self._protocol == "0MQ":
            endpoint_recv = params.get("endpoint_recv")
            endpoint_send = params.get("endpoint_send")
            if endpoint_recv is None or endpoint_send is None:
                raise ValueError("For 0MQ, 'endpoint_recv' and 'endpoint_send' must be provided in params.")
            
            self._endpoint_recv = endpoint_recv
            self._endpoint_send = endpoint_send

        else:
            raise ValueError("Unsupported protocol. Use 'TCP', 'UDS' or 0MQ.")

        self._zmq_socket_recv = self._zmq_context.socket(zmq.PULL)
        self._zmq_socket_recv.connect(self._endpoint_recv)
        self._zmq_socket_send = self._zmq_context.socket(zmq.PUSH)
        self._zmq_socket_send.connect(self._endpoint_send)
        self._running = True
        self._recv_thread = threading.Thread(target=self._recv_loop, daemon=True)
        self._recv_thread.start()

    def disconnect(self):
        """Closes the communication channel."""
        self._running = False
        if self._recv_thread:
            self._recv_thread.join()
        if self._zmq_socket_recv:
            try:
                self._zmq_socket_recv.close()
            except Exception:
                pass
            self._zmq_socket_recv = None
        if self._zmq_socket_send:
            try:
                self._zmq_socket_send.close()
            except Exception:
                pass
            self._zmq_socket_send = None
        if self._zmq_context:
            self._zmq_context.term()
            self._zmq_context = None

    def _recv_loop(self):
        """
        Background thread that waits for incoming messages.
        When data is received, it is deserialized (with msgpack) and immediately pushed
        to the proper stream based on its type.
        """
        unpacker = msgpack.Unpacker(raw=False)
        while self._running:
            try:
                data = self._zmq_socket_recv.recv()
                if not data:
                    break
                unpacker.feed(data)
                for msg in unpacker:
                    self._handle_incoming_message(msg)
            except Exception as e:
                print(e)
                break

    def _handle_incoming_message(self, msg: Dict[str, Any]):
        """
        Dispatches incoming messages to the proper Subject stream based on the message type.
        Expected message structure:
            {
                "type": int,  # feedback = 1, states = 2, calc_states = 10 
                "name_publisher": string,
                "payload": json
            }
        """
        msg_type = msg.get("type")
        if msg_type == 1:
            # Feedback message
            self._feedback_subject.on_next(msg["payload"])
        elif msg_type == 2:
            # State messages
            self._state_subject.on_next(msg["payload"])
        else:
            # Unknown type; optionally log or ignore.
            pass

    def _send_message(self, message: Dict[str, Any]):
        """
        Serializes the given message with msgpack and sends it via the active channel.
        """
        data = msgpack.packb(message, use_bin_type=True)
        if not self._zmq_socket_send:
            raise RuntimeError("Not connected. Please call connect() first.")
        self._zmq_socket_send.send(data)
        

    def send_velocity(self, vel: List[float], name: str):
        payload = {"joint_values": vel}
        message = {
            "type": 31,
            "name_publisher": name,
            "payload": payload
        }
        self._send_message(message)

    def send_position(self, pos: List[float], name: str):
        payload = {"joint_values": pos}
        message = {
            "type": 30,
            "name_publisher": name,
            "payload": payload
        }
        self._send_message(message)

    def send_effort(self, eff: List[float], name: str):
        payload = {"joint_values": eff}
        message = {
            "type": 32,
            "name_publisher": name,
            "payload": payload
        }
        self._send_message(message)

    def send_trajectory(self, trajPoints: List[TrajPoint], name: str, joint_names: List[str]):
        joint_traj_points = []
        for tp in trajPoints:
            traj_dict = {
                "positions": tp.positions if tp.positions else [],
                "velocities": tp.velocities if tp.velocities else [],
                "accelerations": tp.accelerations if tp.accelerations else [],
                "effort": tp.effort if tp.effort else [],
                "seconds": tp.seconds,
                "nanoseconds": tp.nanoseconds
            }
            joint_traj_points.append(traj_dict)
        payload = {"joint_traj_points": joint_traj_points, "joint_names": joint_names}
        message = {
            "type": 20,
            "name_publisher": name,
            "payload": payload
        }
        self._send_message(message)
        
    def send_joypad(self, buttons: List[int], axes: List[float], name: str):
        payload = {"buttons": buttons, "axes": axes}
        message = {
            "type": 50,
            "name_publisher": name,
            "payload": payload
        }
        self._send_message(message)

    def get_feedback_stream(self) -> Subject:
        return self._feedback_subject

    def get_state_stream(self) -> Subject:
        return self._state_subject
