# PX4-Gazebo ROS 2 Simulation Workspace

This workspace is configured for PX4 Autopilot (v1.16.1) simulation with ROS 2 Jazzy and Gazebo.

## Structure
- `src/`: ROS 2 packages source directory.
- `PX4-Autopilot/`: (Ignored by git) PX4 firmware source.

## Setup & Build

1. **Build the workspace:**
   ```bash
   colcon build
   ```

2. **Source the workspace:**
   ```bash
   source install/setup.bash
   ```

## PX4 Simulation
Ensure you have the necessary dependencies installed for PX4 and Gazebo.
To run a basic simulation (example):
```bash
cd PX4-Autopilot
make px4_sitl gz_x500
```
