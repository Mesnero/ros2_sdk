# ROS2 PTYHON SDK

A Python SDK for the [ROS2 API](https://github.com/Mesnero/ros2-api), providing methods for sending velocity, position, effort, and trajectory commands over TCP or UNIX domain sockets or ZeroMQ. The SDK uses MessagePack for serialization and RxPY for reactive feedback and state streams.

Please refer to the [ROS2 API](https://github.com/Mesnero/ros2-api) for a more in depth explanation about message structures and the protocols.

## Features

- **Connection Methods:**  
  Connect using TCP, UNIX domain sockets or ZeroMQ.
  Important: To have a connection to the ros2-api, the receiving endpoint in the SDK must match the sending endpoint in ROS2. (Same for receiving)
  
- **Command Methods:**  
  - `send_velocity(vel: List[float], name: str)`
  - `send_position(pos: List[float], name: str)`
  - `send_effort(eff: List[float], name: str)`
  - `send_trajectory(trajPoints: List[TrajPoint], name: str)`
  - `send_joypad(buttons: List[int], axes: List[float])`

  
- **Reactive Streams:**  
  Subscribe to feedback and state streams using RxPY Subjects:
  - `get_feedback_stream() -> Subject`
  - `get_state_stream() -> Subject`

## Installation

Installation from source:
```bash
git clone https://github.com/yourusername/ros2_sdk.git
cd ros2_sdk
pip install -e .
```
## Usage
```python
from ros2_sdk import ROS2SDK, TrajPoint

# Initialize and connect
sdk = ROS2SDK()
sdk.connect("TCP", {"ip": "127.0.0.1", "port_send": 5555, "port_recv": 5556})
sdk.connect("UDS", {"path_recv": "/your_path1.socket", "path_send": "/your_path2.socket"})
sdk.connect("0MQ, {"endpoint_send": "tcp://127.0.0.1:5555", "endpoint_recv": "tcp://127.0.0.1:5556"})

# Send a command
sdk.send_velocity([1.0, 2.0, 3.0], "velocity_publisher")

# Subscribe to feedback and state streams
sdk.get_feedback_stream().subscribe(lambda msg: print("Feedback:", msg))
sdk.get_state_stream().subscribe(lambda msg: print("State:", msg))

```

## License
This project is licensed under the MIT License - see the [LICENSE](https://github.com/Mesnero/ros2_sdk/blob/main/LICENSE) file for details.

