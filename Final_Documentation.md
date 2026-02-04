<div align="center">

# ARI3215: Robotics Assignment
## Project: Sort-a-bot

**Team Members:**
1. Ellyn Rose Debrincat
2. Joachim Grech
3. Benjamin Zammit (last and certainly least)

---
*University of Malta | B.Sc. AI | February 2026*

</div>

\pagebreak

# Table of Contents
1. [Introduction](#introduction)
2. [System Architecture](#system-architecture)
3. [Installation & Setup](#installation--setup)
4. [Simulation Environment (Gazebo)](#simulation-environment-gazebo)
5. [Navigation Methods](#navigation-methods)
    - [LIDAR A* Navigator](#lidar-a-navigator)
    - [Reinforcement Learning Bridge](#reinforcement-learning-bridge)
6. [Troubleshooting](#troubleshooting)

---

# Introduction
Sort-a-bot is an autonomous robotic system designed to navigate a dynamic arena, locate objects (dumbbells), and transport them to designated locations. This project implements a full ROS 2 integration with a physics-based Gazebo Harmonic simulation.

# System Architecture
The system is divided into three primary ROS 2 packages:
- **`sortabot_description`**: Contains the URDF/Xacro models, inertia parameters, and Gazebo plugins.
- **`sortabot_simulation`**: Manages the Gazebo worlds, launch files, and bridge configurations.
- **`sortabot_actions`**: Contains the logical "brain" of the robot, including PID controllers, action servers, and navigation scripts.

# Installation & Setup
### Prerequisites
- ROS 2 (Jazzy or Harmonic)
- Gazebo Sim (Harmonic)
- Python Dependencies:
  ```bash
  pip3 install opencv-python PyYAML numpy
  ```

### Build Instructions
```bash
cd ~/ros2_ws
rm -rf build/ install/ log/  # Clean build recommended
colcon build --symlink-install
source install/setup.bash
```

# Simulation Environment (Gazebo)
Our simulation features a balanced, weighted robot model (5kg) with optimized inertia tensors to ensure stability during high-speed turns.

**To launch the simulation:**
```bash
ros2 launch sortabot_simulation sortabot.launch.py
```

# Navigation Methods

### 1. LIDAR A* Navigator
A robust pathfinding implementation using an occupancy grid generated from live LIDAR sweeps and the A* search algorithm for deterministic goal reaching.

**How to run:**
```bash
# In a new terminal
cd ~/ros2_ws/src/robotics-assignment/sortabot_actions/scripts
python3 lidar_navigator.py
```

### 2. Reinforcement Learning Bridge
A lightweight PPO-trained inference bridge using a Numpy-only implementation for high-speed decision making without heavy ML dependencies on the VM.

**How to run:**
1. **Start the Controller**:
   ```bash
   ros2 run sortabot_actions action_server.py
   ```
2. **Start the RL Brain**:
   ```bash
   python3 ~/ros2_ws/src/robotics-assignment/sortabot_actions/scripts/drive_robot_numpy.py
   ```

# Troubleshooting
- **Robot Tipping**: The URDF has been weighted to 5kg with a low Center of Mass. Ensure `sortabot.urdf.xacro` is the latest version from GitHub.
- **Line Endings**: All scripts have been standardized to `LF` for Linux compatibility.
- **Odom Stale Warning**: If seen in the Action Server, ensure the `ros_gz_bridge` is running correctly (launched automatically via the main launch file).

---
*Documentation finalized on 2026-02-04.*
