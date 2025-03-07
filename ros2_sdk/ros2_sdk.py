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
        # Normal socket attributes for TCP/UDS.
        self._socket: Optional[socket.socket] = None
        # ZeroMQ specific attributes.
        self._zmq_context: Optional[zmq.Context] = None
        self._zmq_socket: Optional[zmq.Socket] = None
        self._protocol: Optional[str] = None  # "TCP", "UDS", or "ZMQ"

        self._recv_thread: Optional[threading.Thread] = None
        self._running: bool = False

        # Subjects for reactive streams.
        self._feedback_subject: Subject = Subject()
        self._state_subject: Subject = Subject()

    def connect(self, protocol: str, params: Dict[str, Any]):
        """
        Connects to the communication channel.
        For TCP, params should include: 'ip' and 'port'
        For UDS, params should include: 'path'
        For ZMQ, params should include: 'endpoint'
        """
        self._protocol = protocol.upper()
        if self._protocol == "TCP":
            ip = params.get("ip")
            port = params.get("port")
            if ip is None or port is None:
                raise ValueError("For TCP, 'ip' and 'port' must be provided in params.")
            self._socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self._socket.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            self._socket.connect((ip, port))
        elif self._protocol == "UDS":
            if platform.system() == "Windows":
                raise ValueError("UDS is not supported on Windows.")
            path = params.get("path")
            if path is None:
                raise ValueError("For UDS, 'path' must be provided in params.")
            self._socket = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
            self._socket.connect(path)
        elif self._protocol == "ZMQ":
            endpoint = params.get("endpoint")
            if endpoint is None:
                raise ValueError("For ZMQ, 'endpoint' must be provided in params.")
            self._zmq_context = zmq.Context()
            self._zmq_socket = self._zmq_context.socket(zmq.PAIR)
            # In this example, the SDK always binds if using ZMQ.
            self._zmq_socket.connect(endpoint)
        else:
            raise ValueError("Unsupported protocol. Use 'TCP', 'UDS', or 'ZMQ'.")

        self._running = True
        self._recv_thread = threading.Thread(target=self._recv_loop, daemon=True)
        self._recv_thread.start()

    def disconnect(self):
        """Closes the communication channel."""
        self._running = False
        if self._recv_thread:
            self._recv_thread.join()
        if self._protocol in ("TCP", "UDS") and self._socket:
            try:
                self._socket.close()
            except Exception:
                pass
            self._socket = None
        elif self._protocol == "ZMQ" and self._zmq_socket:
            try:
                self._zmq_socket.close()
            except Exception:
                pass
            self._zmq_socket = None
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
                if self._protocol in ("TCP", "UDS"):
                    data = self._socket.recv(4096)
                elif self._protocol == "ZMQ":
                    # For ZeroMQ, recv() returns bytes.
                    data = self._zmq_socket.recv()
                else:
                    break

                if not data:
                    # Connection closed.
                    break
                unpacker.feed(data)
                for msg in unpacker:
                    self._handle_incoming_message(msg)
            except Exception as e:
                # Optionally log the error; for now, we simply break the loop.
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
            self._feedback_subject.on_next(msg)
        elif msg_type == 2:
            # State messages
            self._state_subject.on_next(msg)
        else:
            # Unknown type; optionally log or ignore.
            pass

    def _send_message(self, message: Dict[str, Any]):
        """
        Serializes the given message with msgpack and sends it via the active channel.
        """
        data = msgpack.packb(message, use_bin_type=True)
        if self._protocol in ("TCP", "UDS"):
            if not self._socket:
                raise RuntimeError("Not connected. Please call connect() first.")
            self._socket.sendall(data)
        elif self._protocol == "ZMQ":
            if not self._zmq_socket:
                raise RuntimeError("Not connected. Please call connect() first.")
            self._zmq_socket.send(data)
        else:
            raise RuntimeError("Unsupported protocol.")

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

    def send_trajectory(self, trajPoints: List[TrajPoint], name: str):
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
        payload = {"joint_traj_points": joint_traj_points}
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
