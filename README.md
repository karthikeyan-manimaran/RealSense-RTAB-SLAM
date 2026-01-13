# 3D-SLAM Using Intel RealSense, RTAB-Map and ROS 2

## Overview

This repository contains a complete **ROS 2-based 3D SLAM workspace** built around **Intel RealSense RGB-D cameras**, **RTAB-Map**, and **MAVROS**.
The system is designed to enable **real-time localization and dense 3D mapping** for mobile robots and aerial vehicles operating in **unknown or GPS-denied environments**.

The workspace integrates depth sensing, visual odometry, inertial data, and MAVLink telemetry to estimate the robot’s pose while constructing a globally consistent 3D representation of the environment. This enables autonomous navigation, mapping, and environment understanding for robotics and inspection platforms.

---

## Core Capabilities

* Real-time RGB-D SLAM
* Visual-inertial odometry
* Loop-closure and global map optimization
* Dense 3D point-cloud generation
* Occupancy grid and depth map creation
* MAVLink-based vehicle integration (PX4 / ArduPilot)
* ROS 2 compatible modular architecture

---

## System Architecture

```
Intel RealSense Camera
        |
        v
realsense2_camera (ROS 2)
        |
        v
RGB-D + IMU Data
        |
        v
RTAB-Map (Visual Odometry + Graph-SLAM)
        |
        +---- 3D Point Cloud Map
        +---- Occupancy Grid
        +---- TF (Robot Pose)
        |
        v
MAVROS / Robot Controller
```

The RealSense camera provides synchronized RGB, depth, and IMU data.
RTAB-Map performs visual odometry and graph-based SLAM.
MAVROS provides telemetry and control integration with drones or ground vehicles.

---

## Technology Stack

| Component       | Purpose                            |
| --------------- | ---------------------------------- |
| ROS 2           | Robot middleware and communication |
| RTAB-Map        | Graph-based visual SLAM            |
| Intel RealSense | RGB-D and IMU sensing              |
| librealsense    | Low-level RealSense drivers        |
| realsense-ros   | ROS 2 RealSense interface          |
| MAVROS          | MAVLink interface for UAVs/UGVs    |
| OpenCV          | Image and feature processing       |
| vision_opencv   | ROS-OpenCV bridge                  |

---

## Repository Structure

```
RealSense-RTAB-SLAM/
│
├── librealsense/        Intel RealSense SDK
├── realsense-ros/      ROS 2 drivers for RealSense cameras
├── rtabmap/            RTAB-Map SLAM core
├── rtabmap_ros/        RTAB-Map ROS 2 integration
├── mavros/             MAVLink ROS 2 interface
├── vision_opencv/      OpenCV integration for ROS 2
└── README.md
```

This repository intentionally contains **only source code**.
Generated build files (`build/`, `install/`, `log/`) are excluded.

---

## Supported Hardware

* Intel RealSense depth cameras (D435, D455, D415, L515, etc.)
* Mobile robots and UAVs running PX4 or ArduPilot
* Systems with Linux or ROS 2-supported environments

---

## Installation

### 1. Create a ROS 2 workspace

```
mkdir -p slam_ws/src
cd slam_ws/src
git clone https://github.com/karthikeyan-manimaran/RealSense-RTAB-SLAM.git .
```

### 2. Install dependencies

Install RealSense drivers:

```
sudo apt install ros-<ros2-distro>-realsense2-camera
sudo apt install ros-<ros2-distro>-rtabmap-ros
sudo apt install ros-<ros2-distro>-mavros
```

Install MAVROS dependencies:

```
sudo apt install geographiclib-tools
sudo geographiclib-get-geoids egm96-5
```

Replace `<ros2-distro>` with your ROS 2 distribution (foxy, humble, iron, etc.).

---

### 3. Build the workspace

```
cd ~/slam_ws
colcon build
source install/setup.bash
```

---

## Running the System

### Start RealSense camera

```
ros2 launch realsense2_camera rs_launch.py
```

### Start RTAB-Map SLAM

```
ros2 launch rtabmap_ros rtabmap.launch.py
```

### Start MAVROS (if using UAV or rover)

```
ros2 launch mavros mavros.launch.py
```

---

## Visualization

Use RViz2 to view:

* 3D point clouds
* Robot pose (TF)
* Map and occupancy grid

```
rviz2
```

Add:

* `/rtabmap/cloud_map`
* `/rtabmap/grid_map`
* TF frames

---

## Outputs

The system produces:

* Dense 3D maps
* Loop-closed optimized maps
* Robot pose and trajectory
* Depth and RGB streams
* Occupancy grids for navigation

These outputs can be used for autonomous navigation, obstacle avoidance, and exploration.

--- 

## Project Demo

Click to watch the real-time 3D SLAM system running:


media/preview.mp4

---

## Applications

* Autonomous indoor robots
* Drone-based 3D inspection
* Warehouse mapping
* Search and rescue robots
* Infrastructure and tunnel mapping
* Research in robotics and perception

---

## Development Notes

* This repository is designed to be built using `colcon`
* No precompiled binaries are stored
* Submodules are not used; all packages are included as normal source code
* Compatible with simulation or real hardware

---

## License

Each package retains its original open-source license.
Refer to individual package directories for specific license terms.

---






