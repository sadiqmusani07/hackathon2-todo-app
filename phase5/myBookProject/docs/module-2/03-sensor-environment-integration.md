---
sidebar_position: 3
title: "Chapter 3: Sensor and Environment Integration"
description: "Learn to connect simulated sensors to ROS 2 nodes and create complete digital twin systems"
---

# Chapter 3: Sensor and Environment Integration

## Overview

In this final chapter, we'll connect the physics simulation in Gazebo with the visualization in Unity to create a complete digital twin system. You'll learn how to connect simulated sensors to ROS 2 nodes, handle data streams for AI agents, synchronize the two systems, and prepare for AI perception and navigation.

## Connecting Simulated Sensors to ROS 2 Nodes

### Understanding the ROS 2 Bridge Architecture

The digital twin system connects three components:
- **Gazebo Simulation**: Provides physics and sensor data
- **ROS 2 Network**: Transports data between components
- **Unity Visualization**: Provides high-fidelity rendering

### Setting up ros_gz_bridge

The `ros_gz_bridge` connects ROS 2 topics to Gazebo transport:

```bash
# Install the bridge package
sudo apt install ros-humble-ros-gz-bridge

# Run the bridge with a configuration file
ros2 run ros_gz_bridge parameter_bridge __params:=path/to/bridge_config.yaml
```

Example bridge configuration (`bridge_config.yaml`):

```yaml
/**:
  ros_gz_bridge:
    ros__parameters:
      config_file: "bridge_config.yaml"

# Bridge configuration
- ros_topic_name: "/cmd_vel"
  gz_topic_name: "/robot/cmd_vel"
  ros_type_name: "geometry_msgs/msg/Twist"
  gz_type_name: "gz.msgs.Twist"
  direction: ROS_TO_GZ

- ros_topic_name: "/scan"
  gz_topic_name: "/lidar/scan"
  ros_type_name: "sensor_msgs/msg/LaserScan"
  gz_type_name: "gz.msgs.LaserScan"
  direction: GZ_TO_ROS

- ros_topic_name: "/imu/data"
  gz_topic_name: "/imu/data"
  ros_type_name: "sensor_msgs/msg/Imu"
  gz_type_name: "gz.msgs.IMU"
  direction: GZ_TO_ROS
```

### Setting up Unity ROS TCP Connector

Connect Unity to ROS 2 using the TCP connector:

1. **Configure ROS TCP Connector in Unity**:
   - Add ROS Settings component to your scene
   - Set ROS IP to your ROS 2 master (usually `127.0.0.1`)
   - Set port to match your ROS 2 bridge (default `10000`)

2. **Create publisher/subscriber scripts in Unity**:

```csharp
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.MessageTypes.Sensor;
using Unity.Robotics.ROSTCPConnector.MessageTypes.Geometry;

public class SensorBridge : MonoBehaviour
{
    ROSConnection ros;
    string lidarTopic = "unity/lidar_scan";
    string imuTopic = "unity/imu_data";

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<LaserScanMsg>(lidarTopic);
        ros.RegisterPublisher<ImuMsg>(imuTopic);
    }

    void PublishLidarData()
    {
        var lidarMsg = new LaserScanMsg();
        // Populate lidar message with Unity-generated data
        ros.Publish(lidarTopic, lidarMsg);
    }

    void PublishImuData()
    {
        var imuMsg = new ImuMsg();
        // Populate IMU message with Unity-generated data
        ros.Publish(imuTopic, imuMsg);
    }
}
```

### Sensor Data Types and Formats

Common sensor data formats for digital twin integration:

1. **LaserScan** (`sensor_msgs/LaserScan`):
   - `ranges`: Array of distance measurements
   - `intensities`: Array of intensity values
   - `angle_min`, `angle_max`: Angular range
   - `angle_increment`: Angular resolution

2. **PointCloud2** (`sensor_msgs/PointCloud2`):
   - `height`, `width`: Dimensions of point cloud
   - `fields`: Description of point fields (x, y, z, etc.)
   - `data`: Binary data containing points

3. **Image** (`sensor_msgs/Image`):
   - `height`, `width`: Image dimensions
   - `encoding`: Pixel format (rgb8, bgr8, etc.)
   - `data`: Raw image data

4. **Imu** (`sensor_msgs/Imu`):
   - `orientation`: Quaternion representing orientation
   - `angular_velocity`: Angular velocity vector
   - `linear_acceleration`: Linear acceleration vector

## Handling Data Streams for AI Agents

### Data Pipeline Architecture

Design an efficient data pipeline for AI agents:

```
Sensors → Data Collection → Preprocessing → AI Agent → Actions → Actuators
```

### Implementing Data Collection

Create a data collection node that aggregates sensor data:

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Imu, Image
from std_msgs.msg import String
import numpy as np

class DigitalTwinDataCollector(Node):
    def __init__(self):
        super().__init__('digital_twin_data_collector')

        # Subscribe to sensor topics
        self.lidar_sub = self.create_subscription(
            LaserScan, '/lidar/scan', self.lidar_callback, 10)
        self.imu_sub = self.create_subscription(
            Imu, '/imu/data', self.imu_callback, 10)
        self.camera_sub = self.create_subscription(
            Image, '/camera/rgb/image_raw', self.camera_callback, 10)

        # Publisher for processed data
        self.ai_publisher = self.create_publisher(
            String, '/ai_agent/input', 10)

        # Data storage
        self.sensor_data = {
            'lidar': None,
            'imu': None,
            'camera': None
        }

    def lidar_callback(self, msg):
        self.sensor_data['lidar'] = np.array(msg.ranges)
        self.process_and_publish()

    def imu_callback(self, msg):
        self.sensor_data['imu'] = {
            'orientation': [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w],
            'angular_velocity': [msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z],
            'linear_acceleration': [msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z]
        }
        self.process_and_publish()

    def camera_callback(self, msg):
        # Convert image message to numpy array
        # This is a simplified example - actual implementation depends on encoding
        self.sensor_data['camera'] = np.frombuffer(msg.data, dtype=np.uint8)
        self.process_and_publish()

    def process_and_publish(self):
        # Check if all sensor data is available
        if all(data is not None for data in self.sensor_data.values()):
            # Process data for AI agent
            ai_input = self.format_for_ai_agent()

            # Publish to AI agent
            msg = String()
            msg.data = ai_input
            self.ai_publisher.publish(msg)

    def format_for_ai_agent(self):
        # Format sensor data for AI agent consumption
        formatted_data = {
            'lidar_ranges': self.sensor_data['lidar'].tolist(),
            'imu_orientation': self.sensor_data['imu']['orientation'],
            'timestamp': self.get_clock().now().nanoseconds
        }
        return str(formatted_data)

def main(args=None):
    rclpy.init(args=args)
    node = DigitalTwinDataCollector()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Data Preprocessing for AI

Preprocess sensor data to make it suitable for AI agents:

```python
import numpy as np
from scipy import ndimage

class SensorPreprocessor:
    def __init__(self):
        self.lidar_range_min = 0.1
        self.lidar_range_max = 10.0

    def preprocess_lidar(self, lidar_ranges):
        """Preprocess LiDAR data for AI agent"""
        # Replace invalid ranges with max range
        ranges = np.array(lidar_ranges)
        ranges[ranges < self.lidar_range_min] = self.lidar_range_max
        ranges[ranges > self.lidar_range_max] = self.lidar_range_max

        # Normalize to [0, 1]
        normalized = ranges / self.lidar_range_max

        # Smooth data to reduce noise
        smoothed = ndimage.gaussian_filter1d(normalized, sigma=1.0)

        return smoothed

    def preprocess_camera(self, image_data, width, height, encoding):
        """Preprocess camera data for AI agent"""
        if encoding == 'rgb8':
            # Reshape and normalize image
            img = np.frombuffer(image_data, dtype=np.uint8)
            img = img.reshape((height, width, 3))
            normalized = img.astype(np.float32) / 255.0
        else:
            # Handle other encodings
            normalized = image_data

        return normalized
```

## Synchronizing Gazebo Physics with Unity Visualization

### Time Synchronization

Ensure both systems operate on the same timeline:

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from builtin_interfaces.msg import Time
from std_msgs.msg import Header
import time

class TimeSynchronizer(Node):
    def __init__(self):
        super().__init__('time_synchronizer')

        # Publisher for synchronized time
        self.time_pub = self.create_publisher(Time, '/sync_time', 10)

        # Timer for time synchronization
        self.timer = self.create_timer(0.01, self.publish_time)  # 100Hz

    def publish_time(self):
        # Get current ROS time
        current_time = self.get_clock().now().to_msg()

        # Publish to Unity via ROS bridge
        self.time_pub.publish(current_time)

def main(args=None):
    rclpy.init(args=args)
    node = TimeSynchronizer()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
```

### State Synchronization

Keep robot states consistent between systems:

```csharp
// Unity script for receiving state updates
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.MessageTypes.Std;

public class StateSynchronizer : MonoBehaviour
{
    ROSConnection ros;
    string stateTopic = "robot_states";

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.Subscribe<Unity.Robotics.ROSTCPConnector.MessageTypes.Std.StringMsg>(stateTopic, OnRobotStateReceived);
    }

    void OnRobotStateReceived(Unity.Robotics.ROSTCPConnector.MessageTypes.Std.StringMsg stateMsg)
    {
        // Parse state data and update Unity objects
        UpdateRobotPosition(stateMsg.data);
    }

    void UpdateRobotPosition(string stateData)
    {
        // Parse JSON or other format containing position data
        // Update robot transforms in Unity
        // Example: {"position": [x, y, z], "rotation": [x, y, z, w]}
    }
}
```

### Data Consistency Strategies

Maintain consistency between physics and visualization:

1. **State Publishing**: Publish physics states from Gazebo to Unity
2. **Interpolation**: Smooth transitions between states
3. **Error Correction**: Correct visualization drift over time

## Preparing Digital Twin for AI Training

### Creating Training Data

Generate diverse training scenarios:

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Image
import numpy as np
import json
import os

class TrainingDataGenerator(Node):
    def __init__(self):
        super().__init__('training_data_generator')

        # Subscriptions
        self.lidar_sub = self.create_subscription(
            LaserScan, '/lidar/scan', self.lidar_callback, 10)
        self.action_sub = self.create_subscription(
            String, '/ai_agent/action', self.action_callback, 10)

        # Storage for training data
        self.episode_data = []
        self.episode_counter = 0

    def lidar_callback(self, msg):
        # Store sensor data with timestamp
        data_point = {
            'timestamp': self.get_clock().now().nanoseconds,
            'lidar_ranges': list(msg.ranges),
            'action': None  # Will be filled when action is received
        }
        self.episode_data.append(data_point)

    def action_callback(self, msg):
        # Fill in the action for the most recent data point
        if self.episode_data:
            self.episode_data[-1]['action'] = msg.data

    def save_episode(self):
        # Save current episode data to file
        filename = f"episode_{self.episode_counter:04d}.json"
        filepath = os.path.join("training_data", filename)

        with open(filepath, 'w') as f:
            json.dump(self.episode_data, f)

        self.episode_data = []
        self.episode_counter += 1
        self.get_logger().info(f"Saved episode to {filename}")
```

### Environment Randomization

Create diverse training environments:

1. **Lighting Variation**: Randomize lighting conditions
2. **Object Placement**: Randomize obstacle positions
3. **Sensor Noise**: Add realistic sensor noise
4. **Physics Variation**: Slightly vary physical parameters

### Performance Optimization for Training

Optimize for high-throughput training:

```bash
# Run multiple simulation instances
# Use headless mode for faster simulation
gzserver --headless-rendering your_world.sdf

# Optimize Gazebo parameters for speed
# Increase max step size and real-time factor
# Reduce visual quality if not needed
```

## Complete Integration Example

### Launch File for Full System

Create a launch file that starts the complete digital twin:

```python
# launch/digital_twin_complete.launch.py
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.event_handlers import OnProcessExit
from launch.actions import TimerAction

def generate_launch_description():
    # Launch Gazebo simulation
    gzserver = ExecuteProcess(
        cmd=['gzserver', '--verbose', 'path/to/digital_twin_world.sdf'],
        output='screen'
    )

    # Launch Gazebo client (optional, for visualization)
    gzclient = ExecuteProcess(
        cmd=['gzclient', '--verbose'],
        output='screen'
    )

    # Launch ROS-Gazebo bridge
    bridge = ExecuteProcess(
        cmd=['ros2', 'run', 'ros_gz_bridge', 'parameter_bridge',
             '--ros-args', '--params-file', 'path/to/bridge_config.yaml'],
        output='screen'
    )

    # Launch Unity application (assuming it connects via TCP)
    unity_sim = ExecuteProcess(
        cmd=['/path/to/unity/build/UnitySimulation'],
        output='screen'
    )

    # Launch data collection node
    data_collector = ExecuteProcess(
        cmd=['ros2', 'run', 'digital_twin_pkg', 'data_collector'],
        output='screen'
    )

    # Launch AI agent
    ai_agent = ExecuteProcess(
        cmd=['ros2', 'run', 'ai_agent_pkg', 'ai_agent'],
        output='screen'
    )

    return LaunchDescription([
        gzserver,
        gzclient,
        # Add delay before starting bridge to ensure Gazebo is ready
        TimerAction(
            period=5.0,
            actions=[bridge]
        ),
        unity_sim,
        data_collector,
        ai_agent
    ])
```

### Monitoring and Debugging

Monitor the integrated system:

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
import time

class DigitalTwinMonitor(Node):
    def __init__(self):
        super().__init__('digital_twin_monitor')

        # QoS profile for monitoring
        qos_profile = QoSProfile(depth=10)

        # Track message rates and latencies
        self.message_stats = {}

    def monitor_topic(self, topic_name, msg_type):
        """Monitor a topic and report statistics"""
        def callback(msg):
            current_time = self.get_clock().now().nanoseconds
            if topic_name not in self.message_stats:
                self.message_stats[topic_name] = {
                    'count': 0,
                    'last_time': current_time,
                    'avg_rate': 0
                }

            stats = self.message_stats[topic_name]
            time_diff = (current_time - stats['last_time']) / 1e9  # Convert to seconds
            stats['count'] += 1
            stats['avg_rate'] = stats['count'] / time_diff if time_diff > 0 else 0
            stats['last_time'] = current_time

            # Log statistics periodically
            if stats['count'] % 100 == 0:
                self.get_logger().info(
                    f"{topic_name}: {stats['avg_rate']:.2f} Hz")

        self.create_subscription(msg_type, topic_name, callback, qos_profile)

def main(args=None):
    rclpy.init(args=args)
    monitor = DigitalTwinMonitor()

    # Monitor key topics
    monitor.monitor_topic('/lidar/scan', LaserScan)
    monitor.monitor_topic('/imu/data', Imu)
    monitor.monitor_topic('/camera/rgb/image_raw', Image)

    try:
        rclpy.spin(monitor)
    except KeyboardInterrupt:
        pass
    finally:
        monitor.destroy_node()
        rclpy.shutdown()
```

## Best Practices and Troubleshooting

### Performance Best Practices

1. **Optimize update rates**: Match simulation rates to required fidelity
2. **Use efficient data structures**: Minimize serialization overhead
3. **Implement data buffering**: Handle timing differences gracefully
4. **Monitor resource usage**: Keep track of CPU, memory, and network usage

### Common Integration Issues

1. **Coordinate system mismatches**: Ensure consistent frame conventions
2. **Timing issues**: Handle differences in update rates
3. **Data type mismatches**: Verify message type compatibility
4. **Network latency**: Account for communication delays

### Validation Strategies

1. **Unit testing**: Test individual components
2. **Integration testing**: Test system-level behavior
3. **Performance testing**: Verify real-time requirements
4. **Regression testing**: Ensure changes don't break existing functionality

## Summary

In this chapter, you've learned how to:
- Connect simulated sensors to ROS 2 nodes using bridges
- Handle data streams for AI agents with proper preprocessing
- Synchronize Gazebo physics with Unity visualization
- Prepare the digital twin system for AI training applications
- Implement monitoring and debugging for the integrated system

You now have a complete digital twin system that combines Gazebo physics simulation, Unity high-fidelity rendering, and ROS 2 communication for AI agent development. This system enables safe testing and experimentation with humanoid robots before real-world deployment.

## Navigation

- Previous: [Chapter 2 - High-Fidelity Rendering in Unity](./02-high-fidelity-rendering-unity)
- Back to: [Module Overview](../)