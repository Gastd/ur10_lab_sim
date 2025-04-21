# UR10e Lab Simulation

This is a ROS 2 package designed to simulate a UR10e robot in Gazebo. The package includes a world which spawns the robot on top of a table with some toys. The `src` folder contains example scripts and demo code to move the robot using MoveIt 2.

## Features
- Simulate a UR10e robot in Gazebo.
- Predefined worlds, including:
  - An empty world.
  - A world with a table and toys.
- Example scripts to control the robot using MoveIt 2.

## Prerequisites
- ROS 2 (tested with *Humble*).
- Gazebo (Ignition).
- MoveIt 2.
- Python 3.

## Installation
1. Clone this repository into your ROS 2 workspace:
   ```bash
   cd ~/ur10_ws/src
   git clone <repository-url> ur10_lab_sim
   ```
<!-- 2. Install dependencies:
   ```bash
   rosdep install --from-paths src --ignore-src -r -y
   ``` -->
2. Build the workspace:
   ```bash
   cd ~/ur10_ws
   colcon build
   ```

## Usage

### Launch the Simulation
To launch the simulation with the robot on a table with toys:
```bash
ros2 launch ur10_lab_sim table_simulation.launch.py
```

To launch the simulation in an empty world:
```bash
ros2 launch ur10_lab_sim ur_sim_control.launch.py world_file:=worlds/empty.sdf
```

### Run Example Scripts
The `src` folder contains Python scripts to control the robot:
- **`random_goal.py`**: Moves the robot to a random joint configuration.
- **`scan.py`**: Moves the robot through a predefined set of joint configurations to scan an object.
<!-- - **`scan_fixed_positions.py`**: Demonstrates moving the robot to fixed positions. -->

To run an example script, use:
```bash
ros2 run ur10_lab_sim <script_name>
```
Replace `<script_name>` with the desired script, e.g., `random_goal.py`.

## File Structure
- **`worlds/`**: Contains SDF files for different simulation environments.
- **`launch/`**: Launch files to start the simulation.
- **`src/`**: Example scripts to control the robot.
- **`CMakeLists.txt`** and **`package.xml`**: ROS 2 package configuration files.

<!-- ## License
This package is licensed under the BSD license. See the `LICENSE` file for details. -->

## Maintainer
Gabriel Araujo - [gabriel.fp.araujo@gmail.com](mailto:gabriel.fp.araujo@gmail.com)

*README.md created using Copilot*

