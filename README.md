# A6X Pioneer - Robot Description Package

A comprehensive ROS2 package for the A6X Pioneer robot, providing complete URDF models, visual meshes, MoveIt integration, and hardware control.

## Overview

The **a6x_description** package contains the complete robotic description and setup for the A6X Pioneer robot. This package provides:

- **URDF Models**: Complete kinematic and dynamic descriptions of the robot
- **Mesh Files**: Visual meshes for RViz visualization
- **MoveIt**: Motion planning and inverse kinematics
- **ros2_control**: Integration with real hardware
- **Launch Files**: Ready-to-use launch configurations


## Project Structure

```
a6x_description/
├── urdf/                    # URDF and Xacro files
│   ├── a6x.urdf            # Main URDF file
│   └── a6x_macro.urdf.xacro # Xacro macros for modularity
├── meshes/
│   └── visual/              # Visual meshes for RViz
├── launch/                  # ROS2 launch files
├── config/                  # Configuration files (RViz, controllers)
├── scripts/                 # Utility scripts
└── package.xml             # Package metadata
```

## Features

- **6-DOF Robotic Arm**: Complete kinematic chain definition with joint limits
- **URDF/Xacro Models**: Modular robot description for easy customization
- **RViz Visualization**: High-quality visual representation of the robot
- **Joint Control GUI**: Manual joint manipulation via joint_state_publisher_gui
- **ROS2 TF Integration**: Real-time coordinate frame broadcasting

## Dependencies

- ROS2 Humble
- `robot_state_publisher` - Publishes robot state to TF
- `joint_state_publisher_gui` - GUI for manual joint control
- `rviz2` - ROS2 3D visualization tool
- `ros2_control` - Hardware control framework
- `unitree_hardware` - Unitree motor drivers
- `MoveIt` - Motion planning, collision checking, and inverse kinematics

## Usage

### 1. Visualize the Robot in RViz

```bash
ros2 launch a6x_description a6x_description.launch.py
```

This will start RViz with the A6X Pioneer robot model loaded and ready for inspection.

---

### 2. MoveIt Motion Planning & Inverse Kinematics

```bash
ros2 launch moveit_config demo.launch.py
```

This will start MoveIt with the A6X Pioneer robot model ready to plan and execute motions.

---

### 3. ros2_control Hardware Integration (Sim2Real)

```bash
ros2 launch a6x_ros2_control a6x_robot.launch.py
```

This will start RViz synchronized with the real robot. Control the robot by publishing to the position controller:

```bash
ros2 topic pub /position_controller/commands std_msgs/msg/Float64MultiArray "layout:
  dim: []
  data_offset: 0
data: [2.0]"
```

## Robot Specifications

- **Type**: 6-DOF Articulated Robotic Arm
- **Base**: Fixed to workspace
- **Joint Configuration**: Revolute joints with configurable limits
- **End Effector**: Adapter plate for tool integration
- **Visualization**: RViz compatible with realistic mesh representation

## Related Packages

| Package | Description |
|---------|-------------|
| `a6x_ros2_control` | ROS2 control configuration and launch files |
| `unitree_hardware` | Low-level Unitree motor control and communication |
| `moveit_config` | MoveIt configuration for motion planning |

## Maintainer

**常凱文** (jiaokevinzhang@gmail.com)

📍 ERC Lab, 5th Floor, Science and Technology Building  
🏫 National Taiwan Normal University

For issues, questions, or contributions, feel free to reach out!

