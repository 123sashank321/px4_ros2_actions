# PX4 ROS 2 Actions

A **vehicle-type-aware** ROS 2 Action server system for controlling PX4-based drones (Multicopter, Fixed Wing, and VTOL). This package provides a modular architecture with vehicle-specific strategies for takeoff, landing, and mission execution.

## Overview

This package implements a layered architecture with **adaptive vehicle behaviors**:

1. **Primitive Actions**: Individual ROS 2 Action Servers for specific maneuvers (Arm, Disarm, Takeoff, Land, GoTo, SetMode)
2. **Vehicle Strategy Pattern**: Automatically adapts takeoff/landing logic based on vehicle type (MC/FW/VTOL)
3. **Mission Executor**: High-level Action Server that executes sequences of primitives from YAML files
4. **JSON Configuration**: Single configuration file to switch between vehicle types

## Key Features ✨

- 🚁 **Multicopter (MC)**: Vertical takeoff/landing
- ✈️ **Fixed Wing (FW)**: Runway takeoff with pitch, approach pattern landing
- 🔄 **VTOL**: MC takeoff → automatic transition to FW → FW descent → transition to MC → landing
- ⚙️ **JSON Configuration**: Change vehicle type without code modifications
- 📊 **Rich Feedback**: Real-time altitude, state, and transition status
- 🎯 **Cancellation Support**: Safely abort operations mid-flight

## Prerequisites

- **ROS 2 Humble** (or compatible)
- **PX4 Autopilot** (v1.14+)
- **MicroXRCE-DDS Agent** (running to bridge PX4 and ROS 2)
- **Dependencies**: `px4_msgs`, `action_msgs`, `rclpy`, `ament_index_python`

## Installation

```bash
cd ~/ETS/ws_ros2_action/src
git clone <this-repo-url> px4_ros2_actions
cd ~/ETS/ws_ros2_action
colcon build --packages-select px4_ros2_actions --symlink-install
source install/setup.bash
```

## Configuration

### Vehicle Type Selection

Edit `config/vehicle_config.json` to set your vehicle type:

```json
{
    "vehicle": {
        "type": "VTOL",  // Options: "MC", "FW", "VTOL"
        "mc": {
            "default_takeoff_altitude": 50.0,
            "climb_rate": 2.5,
            "descent_rate": 1.0,
            "altitude_tolerance": 0.3
        },
        "fw": {
            "takeoff_pitch": 15.0,
            "approach_altitude": 50.0,
            "flare_altitude": 10.0,
            "altitude_tolerance": 0.5
        },
        "vtol": {
            "mc_takeoff_altitude": 30.0,
            "transition_altitude": 50.0,
            "auto_transition": true,
            "mc_climb_rate": 2.5,
            "fw_climb_rate": 3.0,
            "transition_timeout": 30.0,
            "altitude_tolerance": 0.5
        }
    }
}
```

**After editing, rebuild**:
```bash
colcon build --packages-select px4_ros2_actions --symlink-install
```

## Usage

### 1. Launch System

Start all action servers:

```bash
ros2 launch px4_ros2_actions mission_stack.launch.py
```

This starts:
- `arm_disarm_server` (Services)
- `takeoff_action_server` (Vehicle-aware)
- `land_action_server` (Vehicle-aware)
- `goto_position_server`
- `set_mode_server`
- `mission_executor_server`

### 2. Arm/Disarm (Services)

```bash
# Arm
ros2 service call /arm_disarm std_srvs/srv/Trigger

# Disarm
ros2 service call /disarm std_srvs/srv/Trigger
```

### 3. Takeoff (Vehicle-Aware)

**Multicopter (MC)**:
```bash
ros2 action send_goal /takeoff px4_ros2_actions/action/Takeoff \
  "{pitch: 0.0, yaw: 0.0, latitude: 0.0, longitude: 0.0, altitude: 50.0}" \
  --feedback
```
→ Vertical climb to 50m

**Fixed Wing (FW)**:
```bash
ros2 action send_goal /takeoff px4_ros2_actions/action/Takeoff \
  "{pitch: 15.0, yaw: 0.0, latitude: 0.0, longitude: 0.0, altitude: 100.0}" \
  --feedback
```
→ Runway takeoff with 15° pitch, climb to 100m

**VTOL**:
```bash
ros2 action send_goal /takeoff px4_ros2_actions/action/Takeoff \
  "{pitch: 0.0, yaw: 0.0, latitude: 0.0, longitude: 0.0, altitude: 100.0}" \
  --feedback
```
→ MC climb to 30m → Auto-transition to FW → FW climb to 100m

### 4. Land (Vehicle-Aware)

**Multicopter (MC)**:
```bash
ros2 action send_goal /land px4_ros2_actions/action/Land \
  "{yaw: 0.0, latitude: 0.0, longitude: 0.0, altitude: 0.0}" \
  --feedback
```
→ Vertical descent to ground

**VTOL** (from FW mode at altitude):
```bash
ros2 action send_goal /land px4_ros2_actions/action/Land \
  "{yaw: 0.0, latitude: 0.0, longitude: 0.0, altitude: 0.0}" \
  --feedback
```
→ FW descent to 50m → Transition to MC → Vertical descent to ground

### 5. Go To Position (NED Frame)

```bash
ros2 action send_goal /goto px4_ros2_actions/action/GoToPosition \
  "{x: 10.0, y: 5.0, z: -5.0, yaw: 0.0}" \
  --feedback
```
*Automatically switches to OFFBOARD mode*

### 6. Set Mode

```bash
# Hold/Loiter
ros2 action send_goal /set_mode px4_ros2_actions/action/SetMode "{mode_name: 'HOLD'}"

# Position mode
ros2 action send_goal /set_mode px4_ros2_actions/action/SetMode "{mode_name: 'POSITION'}"
```

### 7. Execute Mission

```bash
ros2 action send_goal /execute_mission px4_ros2_actions/action/ExecuteMission \
  "{mission_name: 'simple_mission'}" \
  --feedback
```

## Mission File Format (YAML)

Create mission files in `missions/` directory:

```yaml
mission:
  - action: arm
  
  - action: takeoff
    params:
      altitude: 50.0
      
  - action: goto
    params:
      x: 10.0
      y: 0.0
      z: -50.0  # Up is negative Z in NED
      yaw: 0.0
      
  - action: set_mode
    params:
      mode_name: HOLD

  - action: land
```

## Architecture

### Vehicle-Type-Aware Design

```
┌─────────────────────────────────────┐
│  config/vehicle_config.json         │
│  (MC / FW / VTOL)                   │
└──────────────┬──────────────────────┘
               │
               ▼
┌─────────────────────────────────────┐
│  VehicleConfig Loader               │
│  - Reads JSON                       │
│  - Validates vehicle type           │
│  - Provides parameters              │
└──────────────┬──────────────────────┘
               │
               ▼
┌─────────────────────────────────────┐
│  Strategy Pattern                   │
│  ├─ MCStrategy                      │
│  ├─ FWStrategy                      │
│  └─ VTOLStrategy                    │
└──────────────┬──────────────────────┘
               │
               ▼
┌─────────────────────────────────────┐
│  Action Servers                     │
│  ├─ TakeoffActionServer             │
│  └─ LandActionServer                │
└─────────────────────────────────────┘
```

### Core Components

- **`takeoff_action_server.py`**: Vehicle-aware takeoff with strategy delegation
- **`land_action_server.py`**: Vehicle-aware landing with strategy delegation
- **`vehicle_config.py`**: JSON configuration loader
- **`vehicle_strategies/`**:
  - `base_vehicle_strategy.py`: Abstract interface
  - `mc_strategy.py`: Multicopter logic
  - `fw_strategy.py`: Fixed wing logic
  - `vtol_strategy.py`: VTOL transition logic
- **`arm_disarm_server.py`**: Arm/disarm service provider
- **`goto_position_server.py`**: Offboard position control
- **`set_mode_server.py`**: Flight mode management
- **`mission_executor_server.py`**: YAML mission orchestrator

## VTOL Behavior Details

### Takeoff Flow
1. **MC Climb**: 0m → `mc_takeoff_altitude` (30m default)
2. **Mode Switch**: Enter AUTO_LOITER
3. **Transition**: MC → FW (automatic if `auto_transition: true`)
4. **FW Climb**: 30m → target altitude (e.g., 100m)
5. **Complete**: Returns success when at target altitude in FW mode

### Landing Flow
1. **FW Descent**: Current altitude → `transition_altitude` (50m default)
2. **Transition**: FW → MC
3. **MC Descent**: 50m → 0m
4. **Complete**: Returns success when altitude < 0.2m

## Testing

### Test MC Configuration
```bash
# Edit config: "type": "MC"
colcon build --packages-select px4_ros2_actions --symlink-install
ros2 launch px4_ros2_actions mission_stack.launch.py

# Send MC takeoff
ros2 action send_goal /takeoff px4_ros2_actions/action/Takeoff \
  "{pitch: 0.0, yaw: 0.0, latitude: 0.0, longitude: 0.0, altitude: 20.0}" \
  --feedback
```

### Test VTOL Configuration
```bash
# Edit config: "type": "VTOL"
colcon build --packages-select px4_ros2_actions --symlink-install

# Start PX4 SITL with VTOL
cd ~/PX4-Autopilot
make px4_sitl gazebo-classic_standard_vtol

# Start action servers
ros2 launch px4_ros2_actions mission_stack.launch.py

# Test VTOL takeoff with transition
ros2 action send_goal /takeoff px4_ros2_actions/action/Takeoff \
  "{pitch: 0.0, yaw: 0.0, latitude: 0.0, longitude: 0.0, altitude: 100.0}" \
  --feedback
```

## Troubleshooting

**Q: Action servers crash with import errors**  
A: Rebuild with `--symlink-install` flag and source the workspace

**Q: VTOL doesn't transition**  
A: Check that `auto_transition: true` in config and vehicle is in AUTO_LOITER mode

**Q: Transition times out**  
A: Increase `transition_timeout` in VTOL config (default: 30s)

**Q: Vehicle doesn't climb in FW mode after transition**  
A: Ensure target altitude > `mc_takeoff_altitude` to trigger FW climb phase

## Contributing

When adding new vehicle types or strategies:
1. Create new strategy class in `vehicle_strategies/`
2. Inherit from `BaseVehicleStrategy`
3. Implement required methods
4. Add vehicle type to `VehicleType` enum
5. Update `vehicle_config.json` schema

## License

Apache 2.0
