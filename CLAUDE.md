# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

Spot Micro (iKelev Robot Project) — a ROS2 Humble quadruped robot with mixed Python/C++ packages. Targets a Raspberry Pi with PCA9685 servo controller and MPU6050 IMU over I2C.

## Build & Run Commands

```bash
# Build all packages (run from workspace root ~/spot_ws/)
colcon build

# Build a single package
colcon build --packages-select <package_name>

# Source workspace after building
source ~/spot_ws/install/setup.bash

# Launch full system
ros2 launch spot_bringup spot_app.launch.py

# Launch controller input separately (required, not included in spot_app)
ros2 run motion_control controller_input

# Launch individual nodes
ros2 run servo_driver pca9685_server
ros2 run motion_control spot_controller
ros2 launch mpu6050driver mpu6050driver_launch.py

# Visualize URDF in RViz
ros2 launch spotmicro_description display.launch.xml

# Convenience scripts (Linux with gnome-terminal)
./launch_spot_GUI.sh      # Each node in its own terminal
./launch_spot_noGUI.sh    # Background processes
```

## Testing

```bash
# Python packages use pytest via ament (copyright, flake8, pep257)
colcon test --packages-select motion_control
colcon test-result --verbose

# Standalone IMU test
python3 mpu6050driver/mpu6050_test.py
```

## Architecture

### Packages

| Package | Language | Build System | Purpose |
|---|---|---|---|
| `motion_control` | Python | ament_python | Keyboard input + motion controller |
| `servo_driver` | Python | ament_python | PCA9685 servo service server |
| `mpu6050driver` | C++ | ament_cmake | MPU6050 IMU publisher (100 Hz) |
| `spot_interfaces` | - | ament_cmake | ROS2 service definitions (SetServo, StateMachineSet, StateMachineInfo) |
| `spotmicro_description` | - | ament_cmake | URDF/XACRO robot model |
| `state_machine` | C++ | ament_cmake | State management (skeleton, not yet implemented) |
| `spot_bringup` | - | ament_cmake | Launch files for system orchestration |

### ROS2 Communication

**Topics:**
- `control_input` (String) — keyboard commands from `controller_input` to `spot_controller`
- `imu` (sensor_msgs/Imu) — IMU data from `mpu6050driver`

**Services:**
- `set_servo` (SetServo) — set a servo angle by channel number
- `state_machine_info` (StateMachineInfo) — query robot state
- `state_machine_set` (StateMachineSet) — change robot state

### Servo Naming Convention

Format: `XYZZ` where X=L/R, Y=F/R (Front/Rear), ZZ=joint type (EL=elbow, IR=internal rotation, ER=knee).

Channel mapping in `motion_control/motion_control/defines.py`:
- Right Front: K=1, I=3, E=5
- Left Front: K=2, I=4, E=6
- Right Back: K=15, I=13, E=11
- Left Back: K=14, I=12, E=10

### Key Source Files

- `motion_control/motion_control/spot_controller.py` — main motion logic with primitives (wake_up, lie_down, sit, get_up, low)
- `motion_control/motion_control/defines.py` — servo constants, calibration values, position constraints
- `motion_control/motion_control/servo_fun_lib.py` — motion interpolation helpers
- `mpu6050driver/src/mpu6050sensor.cpp` — low-level I2C sensor interface
- `mpu6050driver/params/mpu6050.yaml` — IMU calibration parameters

## Development Notes

- The workspace is expected at `~/spot_ws/` with this repo under `~/spot_ws/src/`
- C++ packages use C++14 minimum (VS Code configured for gnu++17)
- Python targets 3.10+
- `spot_controller` directly uses `adafruit_servokit` — transition to the `set_servo` ROS2 service is in progress but incomplete
- The `state_machine` package is a stub; full implementation is TODO
- IMU driver performs automatic calibration on startup (configurable via YAML params)
