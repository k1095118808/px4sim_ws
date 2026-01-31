# PX4-Gazebo ROS 2 Simulation Workspace

This workspace is configured for PX4 Autopilot (v1.16.1) simulation with ROS 2 Jazzy and Gazebo. It features a reliable offboard control demo, automated split-view launch using Kitty, and a ROS-Gazebo camera bridge.

## Project Structure

```
/home/kxd/ws/px4sim_ws/
├── src/
│   ├── px4_bringup/       # Manages simulation launch + Bridge + Kitty
│   │   ├── launch/sitl.launch.py
│   │   └── scripts/launch_sim.py
│   ├── px4_offboard/      # Offboard control logic node
│   │   ├── px4_offboard/offboard_control.py
│   │   └── launch/offboard.launch.py
│   └── px4_msgs/          # ROS 2 message definitions matching PX4 uORB topics
├── PX4-Autopilot/         # PX4 Firmware Source (Ignored by colcon)
├── launch_session.conf    # Kitty session configuration
└── README.md
```

## Setup & Build

Build the ROS 2 packages (skipping the heavy PX4 firmware):
```bash
colcon build
source install/setup.bash
```

## Running the Simulation

### Option A: Smart Launch (Recommended)
This method automatically detects `kitty` and launches a split-view session with:
1.  **Left Pane**: PX4 Simulation (Interactive `pxh>` shell).
2.  **Right Pane**: Networking (MicroXRCEAgent + ROS-GZ Bridge).

```bash
ros2 run px4_bringup launch_sim
```

### Option B: Manual Launch
If you don't have Kitty or prefer manual terminals:
```bash
ros2 launch px4_bringup sitl.launch.py
```

## Running Offboard Control
Once the simulation is running, launch the control node in a new terminal:
```bash
ros2 launch px4_offboard offboard.launch.py
```
*Beahvior: The drone will arm, takeoff to 5m, and fly a circle of radius 5m.*

## Features
- **Camera Bridge**: RGB and Depth images are bridged to ROS 2:
    - `/camera/image_raw` (RGB)
    - `/camera/depth/image_raw` (Depth)
    - `/camera/camera_info`
- **Interactive Shell**: The smart launch allows typing PX4 commands (e.g., `commander takeoff`) directly in the simulation window.
- **Robust Control**: The offboard node handles `Ctrl+C` gracefully (sending land commands) and uses a state machine for stable takeoff.

## Data Flow Architecture

```mermaid
graph TD
    subgraph "PX4 SITL (Gazebo)"
        PX4[PX4 Autopilot]
        Cam[Camera Sensors]
    end

    subgraph "Bridge"
        Agent[MicroXRCEAgent]
        GZBridge[ROS-GZ Bridge]
    end

    subgraph "ROS 2 Workspace"
        Node[offboard_control Node]
        Rviz[RViz / Image View]
    end

    %% Connections
    PX4 <-->|UDP (uORB)| Agent
    Cam -->|GZ Topics| GZBridge
    Agent <-->|DDS (ROS 2 Topics)| Node
    GZBridge -->|ROS 2 Images| Rviz

    %% Specific Topics
    Node -- Pub: /fmu/in/offboard_control_mode --> Agent
    Node -- Pub: /fmu/in/trajectory_setpoint --> Agent
    Node -- Pub: /fmu/in/vehicle_command --> Agent
    Agent -- Sub: /fmu/out/vehicle_status --> Node
```
