# ROS2 Workspace for Differential Robot
## Overview
  This ROS 2 workspace controls a differential drive robot. The workspace contains packages for robot description (URDF), simulations in Gazebo and RViz, and control functionality. The goal is to provide an environment for testing and controlling a differential drive robot.
### Workspace Structure
  This workspace contains :
```
differential_bot_ws/                # Root directory of the ROS 2 workspace
├── src/                             # Source folder containing the packages
│   ├── bot/                         # Main package for the robot
│   │   ├── launch/                  # Launch files for Gazebo and RViz
│   │   ├── models/                  # Gazebo model files for the robot
│   │   ├── urdf/                    # URDF files for robot description
│   │   ├── world/                   # Gazebo world files for simulation
│   │   ├── CMakeLists.txt           # Build instructions for the package
│   │   └── package.xml              # Package metadata and dependencies
└── ...

```
### Package Details
#### Bot
This package contains everything needed to simulate and control the differential robot. The key components of the package are:
1. **Launch files**: For starting Gazebo and RViz simulations.
2. **Models**: Gazebo-specific model definitions.
3. **URDF**: Robot description for use in Gazebo and RViz.
4. **World**: Custom or default Gazebo worlds to simulate the robot’s interaction with its environment.
