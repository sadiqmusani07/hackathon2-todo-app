---
sidebar_position: 1
title: "Chapter 1: Physics Simulation in Gazebo"
description: "Learn to create realistic physics simulations in Gazebo for humanoid robotics"
---

# Chapter 1: Physics Simulation in Gazebo

![Gazebo Simulation](/img/robot.jpg)

## Overview

Gazebo is a powerful physics simulation environment that provides realistic simulation of robots and their environments. In this chapter, you'll learn how to set up physics simulations with realistic gravity, collisions, and dynamics for humanoid robots.

## Setting Up Gazebo Physics Environment

### Installing Gazebo

Before we begin, ensure you have Gazebo Garden or later installed. For ROS 2 Humble Hawksbill users, you can install the ROS 2 Gazebo packages:

```bash
sudo apt update
sudo apt install ros-humble-gazebo-ros-pkgs ros-humble-gazebo-plugins ros-humble-gazebo-dev
```

### Basic Physics Configuration

Gazebo uses the Ignition Physics engine to simulate realistic physics. The core physics parameters include:

- **Gravity**: Typically set to -9.8 m/s² in the Z direction (downward)
- **Damping**: Reduces velocity over time to simulate friction
- **Real-time factor**: Controls simulation speed relative to real time

### Creating Your First Physics World

Let's start by creating a basic world file with physics properties:

```xml
<?xml version="1.0" ?>
<sdf version="1.7">
  <world name="digital_twin_world">
    <!-- Physics engine configuration -->
    <physics type="ignition-physics_6_0">
      <gravity>0 0 -9.8</gravity>
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>
    </physics>

    <!-- Include a ground plane -->
    <include>
      <uri>model://ground_plane</uri>
    </include>

    <!-- Include a sky -->
    <include>
      <uri>model://sun</uri>
    </include>
  </world>
</sdf>
```

Save this as `digital_twin_world.sdf` in your Gazebo models directory.

## Gravity and Collision Setup

### Understanding Gravity in Gazebo

Gravity in Gazebo is defined as a 3D vector that affects all objects in the simulation. The default value of `0 0 -9.8` means gravity pulls objects downward at 9.8 m/s²:

- X: 0 (no horizontal gravity)
- Y: 0 (no horizontal gravity)
- Z: -9.8 (downward gravity)

### Collision Properties

Each object in Gazebo has collision properties that define how it interacts with other objects:

```xml
<collision name="collision">
  <geometry>
    <box>
      <size>1.0 1.0 1.0</size>
    </box>
  </geometry>
  <surface>
    <friction>
      <ode>
        <mu>1.0</mu>
        <mu2>1.0</mu2>
      </ode>
    </friction>
    <bounce>
      <restitution_coefficient>0.1</restitution_coefficient>
      <threshold>100000</threshold>
    </bounce>
  </surface>
</collision>
```

Key parameters:
- `mu`: Primary friction coefficient
- `mu2`: Secondary friction coefficient
- `restitution_coefficient`: Bounciness (0 = no bounce, 1 = perfectly elastic)

## Robot and Environment Configuration

### Adding a Simple Robot

Let's create a simple robot model with proper physics properties:

```xml
<model name="simple_robot">
  <pose>0 0 0.5 0 0 0</pose>
  <link name="chassis">
    <pose>0 0 0.1 0 0 0</pose>
    <collision name="collision">
      <geometry>
        <box>
          <size>0.5 0.3 0.2</size>
        </box>
      </geometry>
    </collision>
    <visual name="visual">
      <geometry>
        <box>
          <size>0.5 0.3 0.2</size>
        </box>
      </geometry>
    </visual>
    <inertial>
      <mass>1.0</mass>
      <inertia>
        <ixx>0.01</ixx>
        <ixy>0</ixy>
        <ixz>0</ixz>
        <iyy>0.01</iyy>
        <iyz>0</iyz>
        <izz>0.01</izz>
      </inertia>
    </inertial>
  </link>
</model>
```

### Environment Objects

You can add various environment objects to test your robot:

```xml
<!-- Add a box obstacle -->
<model name="obstacle_box">
  <pose>2 0 0.5 0 0 0</pose>
  <link name="link">
    <collision name="collision">
      <geometry>
        <box>
          <size>0.5 0.5 0.5</size>
        </box>
      </geometry>
    </collision>
    <visual name="visual">
      <geometry>
        <box>
          <size>0.5 0.5 0.5</size>
        </box>
      </geometry>
    </visual>
    <inertial>
      <mass>5.0</mass>
      <inertia>
        <ixx>0.208</ixx>
        <ixy>0</ixy>
        <ixz>0</ixz>
        <iyy>0.208</iyy>
        <iyz>0</iyz>
        <izz>0.208</izz>
      </inertia>
    </inertial>
  </link>
</model>
```

## Sensor Simulation

### LiDAR Simulation

Gazebo can simulate various sensors including LiDAR:

```xml
<sensor name="lidar" type="ray">
  <pose>0.2 0 0.1 0 0 0</pose>
  <ray>
    <scan>
      <horizontal>
        <samples>360</samples>
        <resolution>1</resolution>
        <min_angle>-3.14159</min_angle>
        <max_angle>3.14159</max_angle>
      </horizontal>
    </scan>
    <range>
      <min>0.1</min>
      <max>10.0</max>
      <resolution>0.01</resolution>
    </range>
  </ray>
  <plugin name="lidar_controller" filename="libgazebo_ros_ray_sensor.so">
    <ros>
      <namespace>lidar</namespace>
      <remapping>~/out:=scan</remapping>
    </ros>
    <output_type>sensor_msgs/LaserScan</output_type>
  </plugin>
</sensor>
```

### IMU Simulation

Simulate an Inertial Measurement Unit (IMU):

```xml
<sensor name="imu_sensor" type="imu">
  <always_on>true</always_on>
  <update_rate>100</update_rate>
  <pose>0 0 0 0 0 0</pose>
  <plugin name="imu_plugin" filename="libgazebo_ros_imu.so">
    <ros>
      <namespace>imu</namespace>
      <remapping>~/out:=data</remapping>
    </ros>
    <initial_orientation_as_reference>false</initial_orientation_as_reference>
  </plugin>
</sensor>
```

### Depth Camera Simulation

Simulate a depth camera for 3D perception:

```xml
<sensor name="depth_camera" type="depth">
  <pose>0.1 0 0.1 0 0 0</pose>
  <camera>
    <horizontal_fov>1.047</horizontal_fov>
    <image>
      <width>640</width>
      <height>480</height>
      <format>R8G8B8</format>
    </image>
    <clip>
      <near>0.1</near>
      <far>10</far>
    </clip>
  </camera>
  <always_on>true</always_on>
  <update_rate>30</update_rate>
  <visualize>true</visualize>
  <plugin name="camera_controller" filename="libgazebo_ros_openni_kinect.so">
    <ros>
      <namespace>camera</namespace>
      <remapping>~/depth/image_raw:=depth/image_raw</remapping>
      <remapping>~/rgb/image_raw:=rgb/image_raw</remapping>
    </ros>
    <output_type>sensor_msgs/PointCloud2</output_type>
  </plugin>
</sensor>
```

## Physics Parameter Tuning

### Understanding Time Steps

The physics simulation runs in discrete time steps. Key parameters:

- `max_step_size`: The largest time step the simulator will take (typically 0.001s)
- `real_time_update_rate`: How many simulation steps per real second
- `real_time_factor`: Desired speed of simulation vs real time (1.0 = real-time)

### Tuning for Stability

For stable simulation:

1. **Smaller time steps**: More accurate but slower simulation
2. **Appropriate mass and inertia**: Realistic values for stable physics
3. **Collision properties**: Proper friction and bounce values

### Performance vs Accuracy Trade-offs

For educational purposes, balance these settings:

```xml
<physics type="ignition-physics_6_0">
  <gravity>0 0 -9.8</gravity>
  <max_step_size>0.001</max_step_size>  <!-- Smaller for accuracy -->
  <real_time_factor>0.5</real_time_factor>  <!-- Slower for stability -->
  <real_time_update_rate>1000</real_time_update_rate>
</physics>
```

## Running Your Simulation

### Launching Gazebo with Your World

Create a launch file to easily start your simulation:

```python
# launch/digital_twin_sim.launch.py
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.actions import ExecuteProcess

def generate_launch_description():
    world_path = PathJoinSubstitution([
        FindPackageShare('your_robot_description'),
        'worlds',
        'digital_twin_world.sdf'
    ])

    gzserver = ExecuteProcess(
        cmd=['gzserver', '--verbose', world_path],
        output='screen'
    )

    gzclient = ExecuteProcess(
        cmd=['gzclient', '--verbose'],
        output='screen'
    )

    return LaunchDescription([
        gzserver,
        gzclient
    ])
```

### Connecting to ROS 2

Once your simulation is running, you can connect ROS 2 nodes to interact with it:

```bash
# Check available topics
ros2 topic list

# View sensor data
ros2 topic echo /lidar/scan sensor_msgs/msg/LaserScan
ros2 topic echo /imu/data sensor_msgs/msg/Imu
```

## Summary

In this chapter, you've learned how to:
- Set up a Gazebo physics environment with realistic parameters
- Configure gravity, collision properties, and dynamics
- Add robot models and environment objects
- Simulate various sensors (LiDAR, IMU, depth camera)
- Tune physics parameters for stability and performance

In the [next chapter](./02-high-fidelity-rendering-unity), we'll explore how to create high-fidelity visualizations in Unity that match these physics simulations.