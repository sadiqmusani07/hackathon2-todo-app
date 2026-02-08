---
sidebar_position: 3
title: "Module 1, Chapter 3: Humanoid Modeling with URDF"
description: "Define robot bodies for humanoid robotics using URDF"
---

# Humanoid Modeling with URDF

## Role of URDF in Robot Description

URDF (Unified Robot Description Format) is an XML-based format used to describe robot models in ROS. It defines the physical and visual properties of a robot, including its links, joints, inertial properties, and geometric shapes. URDF serves as the robot's "blueprint" that ROS tools use for simulation, visualization, motion planning, and control.

For humanoid robots, URDF becomes particularly important as these robots have complex kinematic structures with multiple degrees of freedom that mimic human anatomy. The URDF description allows ROS tools to understand the robot's structure, enabling proper simulation, collision detection, and motion planning.

### Key Components of URDF:
- **Links**: Rigid bodies that make up the robot structure
- **Joints**: Connections between links that define how they can move relative to each other
- **Visual**: How the robot appears in visualization tools
- **Collision**: How the robot interacts with its environment in simulation
- **Inertial**: Physical properties needed for dynamics simulation

### URDF in the ROS Ecosystem:
- **Robot State Publisher**: Uses URDF to publish transformations between robot parts
- **TF2**: Provides coordinate transforms based on URDF joint states
- **Gazebo**: Uses URDF for physics simulation
- **MoveIt**: Uses URDF for motion planning and collision checking
- **RViz**: Uses URDF for robot visualization

## Links, Joints, and Kinematic Chains

The fundamental building blocks of URDF are links and joints that form kinematic chains. Understanding these components is crucial for modeling humanoid robots with their complex multi-limb structures.

### Links:
Links represent rigid bodies in the robot. Each link has:
- **Visual properties**: How it appears (geometry, material, color)
- **Collision properties**: How it interacts with the environment
- **Inertial properties**: Mass, center of mass, and inertia tensor

```xml
<link name="base_link">
  <visual>
    <geometry>
      <cylinder length="0.6" radius="0.2"/>
    </geometry>
    <material name="blue">
      <color rgba="0 0 .8 1"/>
    </material>
  </visual>
  <collision>
    <geometry>
      <cylinder length="0.6" radius="0.2"/>
    </geometry>
  </collision>
  <inertial>
    <mass value="10"/>
    <origin xyz="0 0 0"/>
    <inertia ixx="1.0" ixy="0.0" ixz="0.0" iyy="1.0" iyz="0.0" izz="1.0"/>
  </inertial>
</link>
```

### Joints:
Joints define how links connect and move relative to each other. Common joint types include:
- **Revolute**: Rotational joint with limited range of motion
- **Continuous**: Rotational joint with unlimited range of motion
- **Prismatic**: Linear sliding joint with limited range of motion
- **Fixed**: No movement between links
- **Floating**: 6 DOF movement (rarely used)

```xml
<joint name="base_to_wheel" type="continuous">
  <parent link="base_link"/>
  <child link="wheel_link"/>
  <origin xyz="0 0.2 0" rpy="0 0 0"/>
  <axis xyz="0 1 0"/>
</joint>
```

### Kinematic Chains:
In humanoid robots, multiple joints and links form kinematic chains that represent limbs:
- **Leg chain**: hip → thigh → knee → shin → foot
- **Arm chain**: shoulder → upper arm → elbow → forearm → hand
- **Spine chain**: pelvis → spine → chest → head

## Humanoid-Specific Design Considerations

Modeling humanoid robots requires special attention to human-like proportions, degrees of freedom, and movement capabilities. These considerations ensure the robot model accurately represents the physical robot and enables realistic simulation.

### Proportional Design:
Humanoid robots should maintain realistic proportions for effective simulation and control:
- **Height ratios**: Typical human proportions (head ~1/8 of total height)
- **Limb ratios**: Arm length ~1.1x height, leg length ~0.9x height
- **Joint ranges**: Match the physical robot's actual range of motion
- **Mass distribution**: Reflect actual weight distribution for dynamics

### Degrees of Freedom (DOF):
Humanoid robots typically have high DOF to enable human-like movement:
- **Head**: 2-3 DOF (pitch, yaw, sometimes roll)
- **Arms**: 6-7 DOF each (shoulder: 3, elbow: 1, wrist: 2-3)
- **Legs**: 6 DOF each (hip: 3, knee: 1, ankle: 2)
- **Torso**: 1-3 DOF (waist rotation, sometimes pitch/yaw)

### Joint Limitations:
Realistic joint limits are crucial for preventing impossible movements:
- **Biological constraints**: Limit ranges to what's physically possible
- **Mechanical constraints**: Respect the physical robot's joint limits
- **Safety margins**: Add safety margins to prevent joint damage

### Example Humanoid Joint Configuration:
```xml
<!-- Humanoid leg with realistic DOF -->
<joint name="left_hip_yaw" type="revolute">
  <parent link="torso"/>
  <child link="left_thigh"/>
  <origin xyz="0 0.1 -0.1" rpy="0 0 0"/>
  <axis xyz="0 0 1"/>
  <limit lower="-0.5" upper="0.5" effort="100" velocity="1.0"/>
</joint>

<joint name="left_hip_pitch" type="revolute">
  <parent link="left_thigh"/>
  <child link="left_shin"/>
  <origin xyz="0 0 -0.4" rpy="0 0 0"/>
  <axis xyz="0 1 0"/>
  <limit lower="-0.2" upper="1.5" effort="100" velocity="1.0"/>
</joint>
```

## How URDF Connects Physical Structure to ROS 2 Controllers

URDF serves as the bridge between the robot's physical description and the control systems in ROS 2. Controllers use the URDF information to understand the robot's structure and apply appropriate control commands.

### Robot State Publication:
The `robot_state_publisher` node uses URDF to publish joint states as TF transforms, allowing other nodes to understand the robot's current configuration in 3D space.

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped

class RobotStatePublisher(Node):
    def __init__(self):
        super().__init__('robot_state_publisher')
        self.joint_state_sub = self.create_subscription(
            JointState, 'joint_states', self.joint_state_callback, 10)
        self.tf_broadcaster = TransformBroadcaster(self)

    def joint_state_callback(self, msg):
        # Process joint states and broadcast transforms based on URDF
        pass
```

### Controller Integration:
Controllers reference URDF joint names and properties to apply control commands:
- **Joint trajectory controllers**: Follow trajectories for specific joints
- **Position controllers**: Control joint positions
- **Velocity controllers**: Control joint velocities
- **Effort controllers**: Control joint forces/torques

### Hardware Interface:
The hardware interface layer maps URDF joint definitions to actual robot hardware, translating ROS 2 commands to physical actuator commands.

## Preparing Robot Models for Simulation and Control

Before using a URDF model in simulation or control, several preparation steps ensure proper functionality and realistic behavior.

### Model Validation:
1. **Syntax validation**: Ensure URDF XML is well-formed and valid
2. **Kinematic validation**: Verify the model has proper kinematic chains
3. **Collision detection**: Check for proper collision geometry
4. **Mass properties**: Verify inertial properties are realistic

### Simulation Preparation:
- **Gazebo plugins**: Add physics and sensor plugins for simulation
- **Material definitions**: Define realistic materials and textures
- **Sensors**: Include sensor models (cameras, IMUs, etc.)
- **Actuators**: Define actuator models with realistic dynamics

### Example Gazebo Integration:
```xml
<!-- Gazebo-specific tags for simulation -->
<gazebo reference="base_link">
  <material>Gazebo/Blue</material>
  <mu1>0.2</mu1>
  <mu2>0.2</mu2>
</gazebo>

<!-- Transmission for hardware interface -->
<transmission name="left_wheel_trans">
  <type>transmission_interface/SimpleTransmission</type>
  <joint name="left_wheel_joint">
    <hardwareInterface>hardware_interface/VelocityJointInterface</hardwareInterface>
  </joint>
  <actuator name="left_wheel_motor">
    <hardwareInterface>hardware_interface/VelocityJointInterface</hardwareInterface>
    <mechanicalReduction>1</mechanicalReduction>
  </actuator>
</transmission>
```

### Control Preparation:
- **Joint limits**: Ensure URDF limits match physical robot capabilities
- **Control parameters**: Define PID gains and other control parameters
- **Safety limits**: Set velocity and acceleration limits for safe operation
- **Calibration**: Include calibration information for accurate control

### Visualization Optimization:
- **Level of detail**: Create simplified versions for real-time visualization
- **Materials and textures**: Use realistic materials for better visualization
- **Performance**: Optimize complex models for real-time rendering

## Learning Objectives

After completing this chapter, you should be able to:
1. Understand the role of URDF in robot description and simulation
2. Create links and joints for kinematic chains in humanoid robots
3. Apply humanoid-specific design considerations to robot models
4. Connect URDF models to ROS 2 controllers and simulation environments
5. Prepare robot models for both simulation and real-world control
6. Validate and optimize URDF models for different applications

## Next Steps

Congratulations! You've completed all three chapters of the ROS 2 Nervous System module. Now you have a comprehensive understanding of:

- ROS 2 core concepts (nodes, topics, services)
- Connecting Python AI agents with rclpy
- Humanoid modeling with URDF

Continue to explore more advanced ROS 2 concepts and consider building your own AI-robotic integration projects.

## Previous Chapters

For a complete understanding of this module, make sure you've reviewed:

- [ROS 2 fundamentals](./ros2-fundamentals) - Core concepts and architecture
- [Python AI integration](./python-ai-integration) - Connecting AI agents to robotics