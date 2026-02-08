---
sidebar_position: 2
title: "Module 1, Chapter 2: Python AI Agents with rclpy"
description: "Connect AI agents to robotic systems using Python and rclpy"
---

# Python AI Agents with rclpy

## Overview of rclpy and Its Role in ROS 2

rclpy is the Python client library for ROS 2, providing a Python API to interact with the ROS 2 ecosystem. It allows Python developers to create ROS 2 nodes, publish and subscribe to topics, provide and call services, and use other ROS 2 features. For AI practitioners familiar with Python, rclpy serves as the bridge between sophisticated AI algorithms and robotic hardware.

rclpy implements the ROS 2 client library interface in Python, wrapping the underlying C library (rcl) with Python bindings. This provides access to all ROS 2 functionality while maintaining the ease of use and rapid development capabilities that Python offers.

### Key Features of rclpy:
- **Node Creation**: Create and manage ROS 2 nodes in Python
- **Topic Communication**: Publish and subscribe to messages
- **Service Communication**: Provide and call services
- **Action Communication**: Work with action-based communication patterns
- **Parameter Management**: Handle node parameters dynamically
- **Time Management**: Work with ROS 2 time and duration concepts

## Structure of a Python-based ROS 2 Node

A typical Python-based ROS 2 node follows a standard structure that includes initialization, setup of publishers/subscribers/services, and a main execution loop or callback-based architecture.

### Basic Node Structure:
```python
import rclpy
from rclpy.node import Node

class MyRobotNode(Node):
    def __init__(self):
        super().__init__('my_robot_node')
        # Initialize publishers, subscribers, services, etc.
        # Setup parameters and timers

    def main_loop(self):
        # Main execution logic (if using loop-based approach)
        pass

def main(args=None):
    rclpy.init(args=args)
    node = MyRobotNode()

    try:
        rclpy.spin(node)  # Keep node alive and processing callbacks
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Publishers in Python:
Publishers are used to send messages to topics. They are created with a specific message type and topic name.

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class AIPublisher(Node):
    def __init__(self):
        super().__init__('ai_publisher')
        self.publisher = self.create_publisher(String, 'ai_commands', 10)
        self.timer = self.create_timer(0.5, self.publish_command)

    def publish_command(self):
        msg = String()
        msg.data = 'AI command: move_forward'
        self.publisher.publish(msg)
```

### Subscribers in Python:
Subscribers receive messages from topics and process them through callback functions.

```python
class AISubscriber(Node):
    def __init__(self):
        super().__init__('ai_subscriber')
        self.subscription = self.create_subscription(
            String,
            'robot_feedback',
            self.feedback_callback,
            10)

    def feedback_callback(self, msg):
        self.get_logger().info(f'Received feedback: {msg.data}')
        # Process the feedback and update AI decision-making
```

## How AI Agents Send Commands and Receive Feedback

The integration of AI agents with robotic systems involves creating communication patterns where AI algorithms can send commands to robots and receive feedback to inform future decisions. This creates a closed-loop control system.

### Command Pipeline:
1. **AI Decision**: The AI algorithm processes sensor data and current state
2. **Command Formation**: Convert AI decisions into ROS 2 messages
3. **Command Publication**: Publish commands to appropriate topics/services
4. **Robot Execution**: Robot hardware/software executes the commands
5. **Feedback Collection**: Collect sensor data and execution results
6. **AI Update**: Update AI state based on feedback

### Example AI Command Structure:
```python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import numpy as np

class AINavigationNode(Node):
    def __init__(self):
        super().__init__('ai_navigation')

        # Publishers for sending commands
        self.cmd_vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10)

        # Subscribers for receiving feedback
        self.scan_subscriber = self.create_subscription(
            LaserScan, 'scan', self.scan_callback, 10)

        # AI model and state
        self.ai_model = self.initialize_ai_model()
        self.robot_state = {'position': None, 'orientation': None}

    def scan_callback(self, msg):
        """Process laser scan data and make navigation decisions"""
        # Process sensor data
        sensor_data = self.process_laser_scan(msg)

        # Run AI decision-making
        ai_decision = self.ai_model.decide(sensor_data, self.robot_state)

        # Convert decision to robot command
        cmd_vel = self.convert_decision_to_command(ai_decision)

        # Publish command
        self.cmd_vel_publisher.publish(cmd_vel)

    def process_laser_scan(self, scan_msg):
        """Convert raw laser scan to processed sensor data"""
        ranges = np.array(scan_msg.ranges)
        # Filter out invalid ranges
        ranges = ranges[np.isfinite(ranges)]
        return ranges

    def convert_decision_to_command(self, decision):
        """Convert AI decision to Twist message"""
        cmd_vel = Twist()
        cmd_vel.linear.x = decision['linear_velocity']
        cmd_vel.angular.z = decision['angular_velocity']
        return cmd_vel
```

## Mapping High-Level AI Decisions to Robot Actions

Converting high-level AI decisions into specific robot actions requires careful consideration of the robot's capabilities, constraints, and the physical world. This mapping process is crucial for effective AI-robot integration.

### Decision Mapping Hierarchy:
1. **Task Level**: High-level goals (e.g., "navigate to location A")
2. **Behavior Level**: Behavioral patterns (e.g., "avoid obstacles while moving")
3. **Action Level**: Specific actions (e.g., "turn left 30 degrees")
4. **Motion Level**: Low-level motion commands (e.g., "set left wheel speed to 0.5 m/s")

### Example Mapping Implementation:
```python
class AIDecisionMapper(Node):
    def __init__(self):
        super().__init__('ai_decision_mapper')

        # Publishers for different command types
        self.nav_publisher = self.create_publisher(PoseStamped, 'goal_pose', 10)
        self.cmd_vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.gripper_publisher = self.create_publisher(GripperCommand, 'gripper_command', 10)

    def map_high_level_decision(self, ai_decision):
        """Map high-level AI decision to specific robot commands"""

        if ai_decision['task'] == 'navigation':
            goal = PoseStamped()
            goal.header.stamp = self.get_clock().now().to_msg()
            goal.header.frame_id = 'map'
            goal.pose.position.x = ai_decision['target_x']
            goal.pose.position.y = ai_decision['target_y']
            self.nav_publisher.publish(goal)

        elif ai_decision['task'] == 'manipulation':
            cmd = self.create_manipulation_command(ai_decision)
            self.gripper_publisher.publish(cmd)

        elif ai_decision['task'] == 'avoid_obstacle':
            cmd_vel = Twist()
            cmd_vel.linear.x = 0.0  # Stop forward motion
            cmd_vel.angular.z = ai_decision['turn_direction'] * 0.5  # Turn to avoid
            self.cmd_vel_publisher.publish(cmd_vel)
```

## Observability, Safety, and Control Flow Considerations

When connecting AI agents to robots, safety and observability are paramount. The system must be designed to handle failures gracefully and provide sufficient monitoring capabilities.

### Safety Considerations:
- **Command Validation**: Validate all AI-generated commands before sending to hardware
- **Emergency Stop**: Implement emergency stop capabilities that override AI commands
- **Safety Boundaries**: Define and enforce physical and operational boundaries
- **Fail-Safe Behaviors**: Define default behaviors when the AI system fails

### Observability Features:
- **Logging**: Comprehensive logging of AI decisions and robot responses
- **Monitoring**: Real-time monitoring of system state and performance metrics
- **Debugging**: Tools to inspect AI decision-making process and intermediate states
- **Telemetry**: Continuous reporting of robot state and sensor data

### Control Flow Patterns:
```python
class SafeAIController(Node):
    def __init__(self):
        super().__init__('safe_ai_controller')

        self.is_emergency_stop = False
        self.ai_active = True

        # Safety monitoring
        self.safety_sub = self.create_subscription(
            SafetyStatus, 'safety_status', self.safety_callback, 10)

        # AI command input
        self.ai_cmd_sub = self.create_subscription(
            AICommand, 'ai_commands', self.ai_command_callback, 10)

        # Robot command output
        self.robot_cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)

    def safety_callback(self, msg):
        """Handle safety status updates"""
        if msg.emergency_stop:
            self.is_emergency_stop = True
            self.stop_robot()
        else:
            self.is_emergency_stop = False

    def ai_command_callback(self, msg):
        """Process AI commands with safety checks"""
        if not self.ai_active or self.is_emergency_stop:
            return

        # Validate command before execution
        if self.validate_command(msg):
            self.robot_cmd_pub.publish(msg.command)
        else:
            self.get_logger().warn(f'Invalid command rejected: {msg.command}')

    def validate_command(self, cmd):
        """Validate command against safety constraints"""
        # Check velocity limits
        if abs(cmd.linear.x) > self.max_linear_velocity:
            return False
        if abs(cmd.angular.z) > self.max_angular_velocity:
            return False
        # Add other validation checks as needed
        return True
```

## Learning Objectives

After completing this chapter, you should be able to:
1. Understand the role and capabilities of rclpy in ROS 2
2. Create Python-based ROS 2 nodes with proper structure
3. Implement AI agents that send commands to robots
4. Design feedback mechanisms for AI-robot communication
5. Map high-level AI decisions to specific robot actions
6. Apply observability and safety considerations to AI-robot systems

## Next Steps

Now that you understand how to connect AI agents with rclpy, continue to the next chapter to learn about [humanoid modeling with URDF](./urdf-modeling).

## Previous Chapter

If you haven't yet, review the [ROS 2 fundamentals](./ros2-fundamentals) chapter to ensure you have the foundational knowledge needed for this content.