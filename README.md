# PX4 ROS 2 Actions (C++)

This package is a C++ port of the original Python `px4_ros2_actions` package. It provides a modular ROS 2 Action Server system for controlling PX4-based drones.

## Overview

This package implements a layered architecture using C++ (`rclcpp` and `rclcpp_action`):
1. **Primitive Actions**: Individual ROS 2 C++ Action Servers for specific maneuvers (`arm_disarm_server`, `takeoff_land_server`, `goto_position_server`, `set_mode_server`).
2. **Mission Executor**: A high-level C++ Action Server that parses a sequence of primitives defined in a YAML file using `yaml-cpp`.

## Prerequisites

- **ROS 2 Humble** (or compatible)
- **PX4 Autopilot** (v1.16)
- **Dependencies**: `px4_msgs`, `yaml-cpp`

*Note: Ensure `yaml-cpp` development packages are installed (e.g. `sudo apt install libyaml-cpp-dev`).*

## Installation

```bash
cd ~/ros2_ws
colcon build --packages-select px4_ros2_actions_cpp
source install/setup.bash
```

## Usage

### 1. Launch System
Start all C++ action servers:

```bash
ros2 launch px4_ros2_actions_cpp mission_stack.launch.py
```

### 2. Run a Mission
Execute a predefined mission from a YAML file (e.g., passing a filename located in `missions/` or an absolute path):

```bash
ros2 action send_goal /execute_mission px4_ros2_actions_cpp/action/ExecuteMission "{mission_name: 'simple_mission'}"
```

### 3. Call Individual Actions

The commands to call individual action servers remain identical to the Python package.

**Arm / Disarm:**
```bash
ros2 action send_goal /arm px4_ros2_actions_cpp/action/Arm "{}"
ros2 action send_goal /disarm px4_ros2_actions_cpp/action/Disarm "{}"
```

**Takeoff (Altitude in meters):**
```bash
ros2 action send_goal /takeoff px4_ros2_actions_cpp/action/Takeoff "{altitude: 5.0}"
```

**Go To Position (NED Frame):**
```bash
ros2 action send_goal /goto px4_ros2_actions_cpp/action/GoToPosition "{x: 10.0, y: 5.0, z: -5.0, yaw: 0.0}"
```

**Set Mode:**
```bash
ros2 action send_goal /set_mode px4_ros2_actions_cpp/action/SetMode "{mode_name: 'POSITION'}"
```

## Architecture

- **`arm_disarm_server.cpp`**: Checks vehicle status and calls arm/disarm commands.
- **`takeoff_land_server.cpp`**: Handles takeoff and landing sequences, monitoring local position.
- **`goto_position_server.cpp`**: Handles Offboard control, streaming `TrajectorySetpoint` messages.
- **`set_mode_server.cpp`**: Safely switches PX4 flight modes mapping string names to command flags.
- **`mission_executor_server.cpp`**: Parses YAML sequences and chains the execution of action clients.
