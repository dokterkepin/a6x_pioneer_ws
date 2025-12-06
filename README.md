# A6X Pioneer - Robot Description Package

A comprehensive ROS2 package for the A6X Pioneer robot, providing complete URDF models, collision meshes, visual meshes, and hardware control integration.

## Overview

The **a6x_description** package contains the complete robotic description and simulation setup for the A6X Pioneer robot. This package provides:

- **URDF Models**: Complete kinematic and dynamic descriptions of the robot
- **Mesh Files**: Visual and collision meshes for simulation and RViz visualization
- **Launch Files**: Ready-to-use launch configurations for visualization and simulation
- **Configuration Files**: Controller and RViz setup configurations

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

- ROS2 (Humble or later)
- `robot_state_publisher` - Publishes robot state to TF
- `joint_state_publisher_gui` - GUI for manual joint control
- `rviz2` - ROS2 3D visualization tool
- `ros2_control` - Hardware control framework (for actual robot control)
- `unitree_hardware` - Motor control drivers

## Usage

### Visualize the Robot in RViz

```bash
ros2 launch a6x_description a6x_description.launch.py
```

This will start RViz with the A6X Pioneer robot model loaded and ready for inspection.

### Load URDF Programmatically

```python
from urdf_parser_py.urdf import URDF

robot = URDF.from_xml_file('urdf/a6x.urdf')
```

### Generate URDF from Xacro

```bash
xacro urdf/a6x_macro.urdf.xacro -o urdf/a6x.urdf
```

## Robot Specifications

- **Type**: 6-DOF Articulated Robotic Arm
- **Base**: Fixed to workspace
- **Joint Configuration**: Revolute joints with configurable limits
- **End Effector**: Adapter plate for tool integration
- **Visualization**: RViz compatible with realistic mesh representation

## Related Packages

- **a6x_ros2_control**: ROS2 control configuration and launch files for actual robot control
- **unitree_hardware**: Low-level motor control and communication
- **MoveIt**: Planned integration for inverse kinematics and motion planning

## Configuration

### RViz Configuration

Default RViz configuration is available at:
```
config/rviz/a6x_config.rviz
```

To use a custom RViz configuration:
```bash
rviz2 -d config/rviz/a6x_config.rviz
```

### Controller Configuration

Motor controller settings are defined in:
```
../a6x_ros2_control/config/controller/a6x_control.yaml
```

## License

See LICENSE file for details.

## Maintainer

**常凱文** (jiaokevinzhang@gmail.com)

For issues, questions, or contributions, please refer to the main project repository.

