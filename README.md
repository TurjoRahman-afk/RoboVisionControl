# ROS2 Workspace

This repository contains ROS2 packages for robot control and camera streaming, including motor control via serial/UDP and OpenCV-based camera publishing.

## Project Structure

- **motor_control**: ROS2 node to control motors and servos via serial and UDP (e.g., ESP32 robot drive).
- **ros2_opencv**: ROS2 nodes for camera publishing and image subscribing using OpenCV.

## Installation

1. **Clone this repository:**
   ```bash
   git clone <repo_url>
   cd ros2_ws
   ```

2. **Set up Python environment and install dependencies:**
   ```bash
   python3 -m venv ros2_venv
   source ros2_venv/bin/activate
   pip install -r requirements.txt
   ```

3. **Build the workspace:**
   ```bash
   source install/setup.bash
   ```

## Usage

### Motor Control Node

```bash
source install/setup.bash
ros2 run motor_control motor_node
```
- Controls motors/servos via serial (`/dev/ttyUSB0`) and UDP (`172.20.10.5:12345`).

### Camera Publisher Node

```bash
source install/setup.bash
ros2 run ros2_opencv camera_publisher
```
- Publishes video frames from a camera (e.g., Arducam IMX477) to `camera/image_raw`.

## Requirements

- ROS2 (Humble/Iron or compatible)
- Python 3.8+
- OpenCV
- cv_bridge
- sensor_msgs, std_msgs, image_transport
- serial

## Contributing

Pull requests and issues are welcome!

## License

Apache License 2.0








   
