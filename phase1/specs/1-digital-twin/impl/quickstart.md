# Quickstart: Digital Twin (Gazebo & Unity) Module

## Prerequisites

Before starting this module, ensure you have:

1. **ROS 2 Humble Hawksbill** (or later) installed and configured
2. **Gazebo Garden** (or later) installed
3. **Unity 2022.3 LTS** (or later) installed with Robotics Package
4. **Basic Python and ROS 2 knowledge** (covered in previous modules)
5. **Node.js v18+** for the Docusaurus documentation

## Setup Environment

### 1. Clone and Navigate to Project
```bash
cd myBookProject
```

### 2. Install Required Dependencies
```bash
npm install
```

### 3. Start Documentation Server
```bash
npm start
```

## Module Overview

This module consists of three interconnected chapters:

1. **Physics Simulation in Gazebo** - Setting up realistic physics environments
2. **High-Fidelity Rendering in Unity** - Creating visual representations
3. **Sensor and Environment Integration** - Connecting systems for digital twin functionality

## Chapter 1: Physics Simulation in Gazebo

### Setup Gazebo Environment
1. Navigate to your Gazebo workspace
2. Create a new simulation world file
3. Configure physics parameters (gravity, damping, etc.)
4. Add robot models to the simulation

### Example: Basic Robot Simulation
```bash
# Launch a simple robot simulation
ros2 launch your_robot_gazebo robot_world.launch.py
```

### Expected Outcome
- Robot model appears in Gazebo environment
- Physics behave realistically (gravity, collisions)
- Robot can be controlled via ROS 2 topics

## Chapter 2: High-Fidelity Rendering in Unity

### Setup Unity Environment
1. Open Unity Hub and create a new 3D project
2. Import the Unity Robotics Package
3. Configure ROS TCP Connector settings
4. Import robot models from URDF or Gazebo

### Example: Robot Visualization
1. Import your robot URDF using the URDF Importer
2. Configure materials and lighting
3. Set up camera views for visualization

### Expected Outcome
- Robot model appears in Unity with proper materials
- Lighting and shadows render realistically
- Robot movements are visualized in real-time

## Chapter 3: Sensor and Environment Integration

### Connect Systems
1. Configure ros_gz_bridge between Gazebo and ROS 2
2. Configure Unity ROS TCP Connector
3. Set up time synchronization
4. Verify data flow between systems

### Example: Sensor Data Flow
```bash
# Launch the complete digital twin setup
ros2 launch digital_twin_bringup complete.launch.py
```

### Expected Outcome
- Sensor data flows from Gazebo simulation to ROS 2
- Unity visualization updates in sync with physics
- AI agents can access both physics and visualization data

## Testing Your Setup

1. Verify Gazebo simulation runs with realistic physics
2. Confirm Unity visualization matches physics behavior
3. Check that sensor data is properly transmitted
4. Test AI agent connection to simulation data

## Troubleshooting Common Issues

### Gazebo Performance
- Reduce physics complexity for slower hardware
- Adjust real_time_factor in simulation configuration

### Unity-ROS Connection
- Verify ROS TCP Connector IP and port settings
- Check firewall settings if connecting across networks

### Synchronization Problems
- Ensure both systems use the same time source
- Adjust buffer sizes for high-frequency data

## Next Steps

After completing this quickstart:
1. Proceed to Chapter 1 for detailed Gazebo setup
2. Move to Chapter 2 for Unity visualization techniques
3. Complete Chapter 3 for full system integration
4. Experiment with your own robot models and scenarios