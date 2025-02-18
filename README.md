# ROS2 PTYHON SDK

A Python SDK for the [ROS2 API](https://github.com/Mesnero/ros2-api), providing methods for sending velocity, position, effort, and trajectory commands over TCP or UNIX domain sockets. The SDK uses MessagePack for serialization and RxPY for reactive feedback and state streams.

Please refer to the [ROS2 API](https://github.com/Mesnero/ros2-api) for a more in depth explanation about message structures and the protocols.

## Features

- **Connection Methods:**  
  Connect using TCP or UNIX domain sockets.
  
- **Command Methods:**  
  - `send_velocity(vel: List[float], name: str)`
  - `send_position(pos: List[float], name: str)`
  - `send_effort(eff: List[float], name: str)`
  - `send_trajectory(trajPoints: List[TrajPoint], name: str)`
  
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
sdk.connect("TCP", {"ip": "127.0.0.1", "port": 12345})

# Send a command
sdk.send_velocity([1.0, 2.0, 3.0], "velocity_publisher")

# Subscribe to feedback and state streams
sdk.get_feedback_stream().subscribe(lambda msg: print("Feedback:", msg))
sdk.get_state_stream().subscribe(lambda msg: print("State:", msg))

# Disconnect when done
sdk.disconnect()
```

## License
This project is licensed under the MIT License - see the [LICENSE](https://github.com/Mesnero/ros2_sdk/blob/main/LICENSE) file for details.

