# APSRC UDP ROSBridge

## Overview
The APSRC UDP ROSBridge is a ROS package designed for efficient communication between APSRC systems and ROS using the UDP protocol. It ensures smooth data transfer, real-time processing, and seamless integration with ROS-based applications.

## Features
- **UDP Communication**: Enables high-speed, low-latency communication.
- **Message Handling**: Translates APSRC-specific packets to ROS messages.
- **Configurable Settings**: Allows IP, port, and other network parameters to be adjusted.
- **Modular Design**: Supports multiple message formats and extensions.

## Installation
### Prerequisites
- **ROS** (Recommended: Noetic or Melodic)
- **Boost Libraries** (for UDP support)
- **C++11 or later**

### Installation Steps
```bash
cd ~/catkin_ws/src
git clone https://github.com/mojtaba1989/apsrc_udp_rosbridge.git
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

## Usage
### Running the ROSBridge Node
```bash
roslaunch apsrc_udp_rosbridge run_all.launch
```

## Code Structure
```
apsrc_udp_rosbridge/
│── include/
│   ├── common.hpp
│   ├── packet_definitions/
│── src/
│   ├── mabx_to_ros.cpp
│   ├── ros_to_mkx.cpp
│   ├── cv_to_ros.cpp
│   ├── ros_to_mabx.cpp
│   ├── here_to_mabx.cpp
│── launch/
│   ├── run_all.launch
│── config/
│   ├── config.yaml
│── CMakeLists.txt
│── package.xml
```

## Source Code Details
### `mabx_to_ros.cpp`
- **Functionality**: Receives UDP messages from MABX and publishes them to ROS topics.
- **Subscribers**: None.
- **Publishers**:
  - `/mabx_commands/velocity_cmd` (`apsrc_msgs::VelocityCommand`)
  - `/mabx_commands/velocity_array_cmd` (`apsrc_msgs::VelocityArrayCommand`)
  - `/mabx_commands/position_cmd` (`apsrc_msgs::PositionCommand`)
  - `/mabx_commands/position_array_cmd` (`apsrc_msgs::PositionArrayCommand`)
  - `/mabx_commands/reset_cmd` (`apsrc_msgs::DriverInputCommand`)

### `ros_to_mkx.cpp`
- **Functionality**: Sends ROS data (GPS, velocity, IMU) to MKX via UDP.
- **Subscribers**:
  - `/gps/gps` (`gps_common::GPSFix`)
  - `/current_velocity` (`geometry_msgs::TwistStamped`)
  - `/current_pose` (`geometry_msgs::PoseStamped`)
- **Publishers**: None.

### `cv_to_ros.cpp`
- **Functionality**: Receives lane centering messages from CV module and publishes them.
- **Subscribers**: None.
- **Publishers**:
  - `/cv_core/offset_to_centerline` (`apsrc_msgs::LaneCentering`)

### `ros_to_mabx.cpp`
- **Functionality**: Sends various vehicle control commands to MABX.
- **Subscribers**:
  - `/base_waypoints` (`autoware_msgs::Lane`)
  - `/closest_waypoint` (`std_msgs::Int32`)
  - `/gps/gps` (`gps_common::GPSFix`)
  - `/lead_vehicle/track` (`apsrc_msgs::LeadVehicle`)
  - `/v2x/SPaTnMAP` (`apsrc_msgs::SPaTnMAP`)
- **Publishers**: None.

### `here_to_mabx.cpp`
- **Functionality**: Converts HERE map data into APSRC-compatible format and sends it to MABX.
- **Subscribers**:
  - `/here/route` (`apsrc_msgs::Response`)
- **Publishers**: None.

## Contribution
Contributions are welcome! Fork the repository, create a new branch, and submit a pull request.

## License
This project is licensed under the MIT License. See [LICENSE](LICENSE) for details.

## Contact
For inquiries, open an issue on GitHub or contact the maintainer.

