import socket
import threading
import msgpack
from typing import List, Dict, Any, Optional
from rx.subject import Subject
from dataclasses import dataclass, field

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
        self._socket: Optional[socket.socket] = None
        self._recv_thread: Optional[threading.Thread] = None
        self._running: bool = False
        # Subjects for reactive streams.
        self._feedback_subject: Subject = Subject()
        self._state_subject: Subject = Subject()

    def connect(self, protocol: str, params: Dict[str, Any]):
        """
        Connects to the socket using TCP or UDS.
        For TCP, params should include: 'ip' and 'port'
        For UDS, params should include: 'path'
        """
        protocol = protocol.upper()
        if protocol == "TCP":
            ip = params.get("ip")
            port = params.get("port")
            if ip is None or port is None:
                raise ValueError("For TCP, 'ip' and 'port' must be provided in params.")
            self._socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self._socket.connect((ip, port))
        elif protocol == "UDS":
            path = params.get("path")
            if path is None:
                raise ValueError("For UDS, 'path' must be provided in params.")
            self._socket = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
            self._socket.connect(path)
        else:
            raise ValueError("Unsupported protocol. Use 'TCP' or 'UDS'.")

        self._running = True
        # Start a background thread that listens for incoming messages.
        self._recv_thread = threading.Thread(target=self._recv_loop, daemon=True)
        self._recv_thread.start()

    def disconnect(self):
        """Closes the socket connection."""
        self._running = False
        if self._socket:
            try:
                self._socket.close()
            except Exception:
                pass
            self._socket = None

    def _recv_loop(self):
        """
        Background thread that waits for incoming messages.
        When data is received, it is deserialized (with msgpack) and immediately pushed
        to the proper stream based on its type.
        """
        unpacker = msgpack.Unpacker(raw=False)
        while self._running:
            try:
                data = self._socket.recv(4096)
                if not data:
                    # Socket closed
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
        elif msg_type in (2, 10):
            # State messages (normal or calculated)
            self._state_subject.on_next(msg)
        else:
            # Unknown type; optionally log or ignore.
            pass

    def _send_message(self, message: Dict[str, Any]):
        """
        Serializes the given message with msgpack and sends it via the socket.
        """
        if not self._socket:
            raise RuntimeError("Not connected. Please call connect() first.")
        data = msgpack.packb(message, use_bin_type=True)
        self._socket.sendall(data)

    def send_velocity(self, vel: List[float], name: str):
        """
        Sends a velocity command. The payload is:
            { "joint_values": [ ... ] }
        and the type is set to 31.
        """
        payload = {"joint_values": vel}
        message = {
            "type": 31,
            "name_publisher": name,
            "payload": payload
        }
        self._send_message(message)

    def send_position(self, pos: List[float], name: str):
        """
        Sends a position command. The payload is:
            { "joint_values": [ ... ] }
        and the type is set to 30.
        """
        payload = {"joint_values": pos}
        message = {
            "type": 30,
            "name_publisher": name,
            "payload": payload
        }
        self._send_message(message)

    def send_effort(self, eff: List[float], name: str):
        """
        Sends an effort command. The payload is:
            { "joint_values": [ ... ] }
        and the type is set to 32.
        """
        payload = {"joint_values": eff}
        message = {
            "type": 32,
            "name_publisher": name,
            "payload": payload
        }
        self._send_message(message)

    def send_trajectory(self, trajPoints: List[TrajPoint], name: str):
        """
        Sends a trajectory command.
        Each TrajPoint is converted into a dictionary with the required keys:
            "positions", "velocities", "accelerations", "effort", "seconds", "nanoseconds"
        The payload structure is:
            { "joint_traj_points": [ { ... }, { ... }, ... ] }
        and the type is set to 20.
        """
        joint_traj_points = []
        for tp in trajPoints:
            traj_dict = {
                "positions": tp.positions,
                "velocities": tp.velocities,
                # If accelerations is empty, you can decide whether to compute it or leave it empty.
                "accelerations": tp.accelerations if tp.accelerations else [],
                "effort": tp.effort,
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

    def get_feedback_stream(self) -> Subject:
        """
        Returns a reactive stream (Subject) where feedback messages (type 1) are pushed.
        The user can subscribe to this Subject to receive messages immediately.
        """
        return self._feedback_subject

    def get_state_stream(self) -> Subject:
        """
        Returns a reactive stream (Subject) where state messages are pushed.
        Both normal (type 2) and calculated states (type 10) are sent here.
        """
        return self._state_subject
