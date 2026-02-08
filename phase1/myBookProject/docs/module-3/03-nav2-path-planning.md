---
sidebar_position: 3
title: "Module 3, Chapter 3: Path Planning with Nav2"
description: "Learn to implement path planning using Nav2 specifically adapted for bipedal humanoid movement patterns and constraints"
---

# Module 3, Chapter 3: Path Planning with Nav2

## Overview

Navigation2 (Nav2) is the premier navigation framework for ROS 2, providing advanced path planning capabilities. This chapter focuses on adapting Nav2 specifically for bipedal humanoid movement patterns and constraints, covering motion planning algorithms, obstacle avoidance, and trajectory control techniques for safe humanoid navigation.

## Nav2 Overview for Bipedal Humanoid Movement

### Introduction to Navigation2

Navigation2 is the successor to ROS Navigation stack, redesigned for ROS 2 with improved flexibility and performance. For humanoid robots, Nav2 provides:

- **Modular Architecture**: Pluggable components for customization
- **Behavior Trees**: Declarative navigation behavior specification
- **Advanced Algorithms**: State-of-the-art motion planning
- **Safety Features**: Built-in obstacle avoidance and recovery

### Nav2 Architecture for Humanoid Robots

```python
# Example: Nav2 architecture for humanoid robots
from nav2_behavior_tree import bt_builder
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Path, OccupancyGrid
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

class HumanoidNav2Manager(Node):
    def __init__(self):
        super().__init__('humanoid_nav2_manager')

        # Nav2 action clients
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # Publishers for humanoid-specific navigation
        self.local_plan_pub = self.create_publisher(Path, '/humanoid/local_plan', 10)
        self.global_plan_pub = self.create_publisher(Path, '/humanoid/global_plan', 10)
        self.velocity_pub = self.create_publisher(Twist, '/humanoid/cmd_vel', 10)

        # Humanoid-specific parameters
        self.humanoid_config = {
            'step_length': 0.4,  # Max step length for humanoid
            'turn_radius': 0.3,  # Minimum turning radius
            'foot_separation': 0.2,  # Distance between feet
            'balance_margin': 0.1,  # Safety margin for balance
            'max_step_height': 0.1  # Max step-over height
        }

    def create_humanoid_navigate_request(self, goal_pose):
        # Create navigation request adapted for humanoid movement
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = goal_pose
        goal_msg.behavior_tree_id = 'humanoid_navigate_bt'  # Custom BT for humanoid

        return goal_msg
```

### Humanoid-Specific Navigation Challenges

Humanoid robots face unique navigation challenges compared to wheeled robots:

- **Balance Constraints**: Maintaining center of mass within support polygon
- **Step Planning**: Discrete foot placement rather than continuous motion
- **Stability Requirements**: Need for stable gait patterns
- **Obstacle Clearance**: Ensuring sufficient clearance for legs and arms
- **Dynamic Balance**: Managing momentum during movement

### Nav2 Components for Humanoid Navigation

```python
# Example: Humanoid-specific Nav2 components
from nav2_core import Planner, Controller, Costmap
from nav2_util.lifecycle_node import LifecycleNode
from geometry_msgs.msg import Point

class HumanoidPathPlanner(LifecycleNode):
    def __init__(self):
        super().__init__('humanoid_path_planner')

    def configure(self, plugin_name, costmap_ros, lifecycle_node, planner_node):
        # Configure for humanoid-specific planning
        self.costmap = costmap_ros.get_costmap()
        self.plugin_name = plugin_name

        # Humanoid-specific parameters
        self.step_length = lifecycle_node.get_parameter(
            'step_length').value
        self.turn_radius = lifecycle_node.get_parameter(
            'turn_radius').value
        self.balance_margin = lifecycle_node.get_parameter(
            'balance_margin').value

    def create_plan(self, start, goal):
        # Plan path considering humanoid kinematic constraints
        # This would implement humanoid-specific path planning algorithms
        humanoid_path = self.plan_humanoid_path(start, goal)

        # Convert to standard Path message
        path_msg = Path()
        path_msg.header.frame_id = 'map'
        path_msg.poses = humanoid_path

        return path_msg

    def plan_humanoid_path(self, start, goal):
        # Implementation of humanoid-aware path planning
        # Consider step constraints, balance polygons, etc.
        planned_path = []

        # Simplified example using RRT for humanoid path planning
        current = start
        while self.distance(current, goal) > 0.5:  # 50cm tolerance
            # Find next valid step position
            next_waypoint = self.find_next_humanoid_step(current, goal)
            planned_path.append(next_waypoint)
            current = next_waypoint

            if len(planned_path) > 1000:  # Prevent infinite loops
                self.get_logger().warn("Path planning exceeded max iterations")
                break

        return planned_path

    def find_next_humanoid_step(self, current, goal):
        # Find next valid position for humanoid foot placement
        # considering balance and obstacle constraints
        direction = self.normalize_vector(
            [goal.pose.position.x - current.pose.position.x,
             goal.pose.position.y - current.pose.position.y]
        )

        # Calculate next step position
        step_x = current.pose.position.x + direction[0] * self.step_length
        step_y = current.pose.position.y + direction[1] * self.step_length

        # Check for balance polygon constraints
        next_pose = PoseStamped()
        next_pose.pose.position.x = step_x
        next_pose.pose.position.y = step_y
        next_pose.pose.position.z = 0.0

        # Check if this step is safe and balanced
        if self.is_safe_for_humanoid(next_pose):
            return next_pose
        else:
            # Find alternative step using local planning
            return self.find_alternative_step(current, goal)
```

## Motion Planning Algorithms for Humanoid Robots

### Overview of Motion Planning Approaches

Different algorithms have varying suitability for humanoid navigation:

1. **A* (A-star)**: Good for static environments, optimal paths
2. **Dijkstra**: Optimal but computationally expensive
3. **RRT (Rapidly-exploring Random Tree)**: Good for high-dimensional spaces
4. **TEB (Timed Elastic Band)**: Real-time optimization for dynamic environments
5. **Footstep Planning**: Specific to legged robots

### Humanoid-Specific Path Planning

```python
# Example: Humanoid-aware path planning algorithm
import numpy as np
from scipy.spatial import KDTree
import math

class HumanoidMotionPlanner:
    def __init__(self):
        self.step_length = 0.4  # meters
        self.turn_radius = 0.3  # meters
        self.balance_margin = 0.1  # meters
        self.max_step_height = 0.1  # meters

    def plan_footsteps(self, start_pose, goal_pose, costmap):
        # Plan sequence of footstep positions for humanoid
        footsteps = [start_pose]

        current_pose = start_pose
        step_count = 0
        max_steps = 100  # Prevent infinite planning

        while self.distance(current_pose, goal_pose) > self.step_length and step_count < max_steps:
            # Find next valid footstep position
            next_step = self.find_next_footstep(current_pose, goal_pose, costmap)

            if next_step is None:
                # Try recovery behavior
                next_step = self.find_alternative_path(current_pose, goal_pose, costmap)

            if next_step is not None:
                footsteps.append(next_step)
                current_pose = next_step
            else:
                # Could not find valid step
                self.get_logger().warn("Could not find valid footstep")
                return None

            step_count += 1

        # Add final step to goal if close enough
        if self.distance(current_pose, goal_pose) <= self.step_length:
            footsteps.append(goal_pose)

        return footsteps

    def find_next_footstep(self, current_pose, goal_pose, costmap):
        # Find next footstep considering humanoid constraints
        candidate_positions = self.generate_step_candidates(current_pose, goal_pose)

        for candidate in candidate_positions:
            if self.is_valid_footstep(candidate, costmap):
                return candidate

        return None

    def generate_step_candidates(self, current_pose, goal_direction):
        # Generate potential footstep positions
        candidates = []

        # Primary direction toward goal
        goal_dir = self.normalize_vector([
            goal_direction.pose.position.x - current_pose.pose.position.x,
            goal_direction.pose.position.y - current_pose.pose.position.y
        ])

        # Forward step
        forward_pos = self.add_vector_to_pose(
            current_pose,
            [goal_dir[0] * self.step_length, goal_dir[1] * self.step_length, 0]
        )
        candidates.append(forward_pos)

        # Lateral steps (left and right)
        left_dir = [-goal_dir[1], goal_dir[0]]  # Rotate 90 degrees left
        right_dir = [goal_dir[1], -goal_dir[0]]  # Rotate 90 degrees right

        left_pos = self.add_vector_to_pose(
            current_pose,
            [left_dir[0] * self.step_length * 0.7, left_dir[1] * self.step_length * 0.7, 0]
        )
        right_pos = self.add_vector_to_pose(
            current_pose,
            [right_dir[0] * self.step_length * 0.7, right_dir[1] * self.step_length * 0.7, 0]
        )

        candidates.extend([left_pos, right_pos])

        # Diagonal steps
        diag_left = self.add_vector_to_pose(
            current_pose,
            [(goal_dir[0] + left_dir[0]) * self.step_length * 0.7,
             (goal_dir[1] + left_dir[1]) * self.step_length * 0.7, 0]
        )
        diag_right = self.add_vector_to_pose(
            current_pose,
            [(goal_dir[0] + right_dir[0]) * self.step_length * 0.7,
             (goal_dir[1] + right_dir[1]) * self.step_length * 0.7, 0]
        )

        candidates.extend([diag_left, diag_right])

        return candidates

    def is_valid_footstep(self, pose, costmap):
        # Check if footstep is valid considering:
        # - Collision with obstacles
        # - Balance constraints
        # - Terrain traversability
        # - Step height constraints

        # Convert pose to costmap coordinates
        mx, my = self.world_to_map(pose.pose.position.x, pose.pose.position.y, costmap)

        # Check costmap for collisions
        if not (0 <= mx < costmap.metadata.size_x and 0 <= my < costmap.metadata.size_y):
            return False  # Outside map bounds

        cost = costmap.data[my * costmap.metadata.size_x + mx]
        if cost >= 253:  # lethal obstacle
            return False

        # Additional checks for humanoid-specific constraints
        if cost > 100:  # consider as invalid if too costly
            return False

        # Check for step height (simplified)
        terrain_height = self.get_terrain_height_at(pose)
        if abs(terrain_height) > self.max_step_height:
            return False

        return True
```

### Gait Pattern Planning

Humanoid robots need to plan gait patterns in addition to paths:

```python
# Example: Gait pattern planning
class GaitPatternPlanner:
    def __init__(self):
        self.stride_length = 0.4  # Distance between consecutive footfalls
        self.step_width = 0.2     # Distance between left and right footprints
        self.step_height = 0.05   # Height of foot lift during stepping
        self.step_duration = 0.8  # Time for each step (seconds)

    def plan_gait_pattern(self, footsteps):
        # Plan gait timing and coordination for footsteps
        gait_sequence = []

        for i, step in enumerate(footsteps[:-1]):  # Skip last step (final pose)
            # Determine which foot makes this step
            support_foot = 'left' if i % 2 == 0 else 'right'
            swing_foot = 'right' if i % 2 == 0 else 'left'

            # Plan step trajectory
            step_trajectory = self.plan_step_trajectory(
                step, footsteps[i+1], swing_foot, self.step_duration
            )

            gait_sequence.append({
                'step_index': i,
                'swing_foot': swing_foot,
                'support_foot': support_foot,
                'trajectory': step_trajectory,
                'start_time': i * self.step_duration,
                'end_time': (i + 1) * self.step_duration
            })

        return gait_sequence

    def plan_step_trajectory(self, start_pose, end_pose, foot, duration):
        # Plan 3D trajectory for a single step
        # This would include lift, swing, and placement phases

        trajectory_points = []
        num_points = int(duration * 50)  # 50Hz trajectory generation

        for i in range(num_points + 1):
            t = i / num_points  # Normalized time (0 to 1)

            # Interpolate position
            x = start_pose.pose.position.x + t * (
                end_pose.pose.position.x - start_pose.pose.position.x
            )
            y = start_pose.pose.position.y + t * (
                end_pose.pose.position.y - start_pose.pose.position.y
            )

            # Add parabolic lift for foot
            lift_factor = 4 * t * (1 - t)  # Parabolic curve
            z = start_pose.pose.position.z + lift_factor * self.step_height

            # Create trajectory point
            traj_point = {
                'position': (x, y, z),
                'time_from_start': rclpy.duration.Duration(seconds=t * duration)
            }
            trajectory_points.append(traj_point)

        return trajectory_points
```

## Obstacle Avoidance and Trajectory Control

### Dynamic Obstacle Avoidance

Humanoid robots need to handle both static and dynamic obstacles:

```python
# Example: Humanoid obstacle avoidance
from geometry_msgs.msg import Point, Vector3
from visualization_msgs.msg import Marker

class HumanoidObstacleAvoider:
    def __init__(self, node):
        self.node = node
        self.obstacles = []  # List of detected obstacles
        self.humanoid_radius = 0.3  # Effective radius for collision checking
        self.avoidance_distance = 0.5  # Minimum safe distance

        # Publishers for visualization
        self.obstacle_marker_pub = self.node.create_publisher(
            Marker, '/humanoid/obstacles', 10
        )

    def update_obstacles(self, detection_msg):
        # Update list of detected obstacles
        self.obstacles = detection_msg.obstacles

    def compute_avoidance_velocity(self, current_pose, current_velocity, goal):
        # Compute avoidance velocity using artificial potential fields
        repulsive_force = self.calculate_repulsive_force(current_pose)
        attractive_force = self.calculate_attractive_force(current_pose, goal)

        # Combine forces
        total_force = self.combine_forces(attractive_force, repulsive_force)

        # Convert to desired velocity
        desired_velocity = self.force_to_velocity(total_force, current_velocity)

        # Ensure humanoid kinematic constraints are satisfied
        constrained_velocity = self.apply_humanoid_constraints(desired_velocity)

        return constrained_velocity

    def calculate_repulsive_force(self, current_pose):
        # Calculate repulsive force from all obstacles
        total_force = np.array([0.0, 0.0])

        for obstacle in self.obstacles:
            # Calculate distance and direction to obstacle
            dx = obstacle.pose.position.x - current_pose.pose.position.x
            dy = obstacle.pose.position.y - current_pose.pose.position.y
            distance = math.sqrt(dx*dx + dy*dy)

            if distance < self.avoidance_distance:
                # Normalize direction away from obstacle
                if distance > 0.01:  # Avoid division by zero
                    dir_x = -dx / distance
                    dir_y = -dy / distance
                else:
                    # Random direction if too close
                    angle = np.random.uniform(0, 2*np.pi)
                    dir_x = np.cos(angle)
                    dir_y = np.sin(angle)

                # Calculate force magnitude (stronger when closer)
                force_magnitude = (1.0/distance - 1.0/self.avoidance_distance) * 5.0
                total_force[0] += dir_x * force_magnitude
                total_force[1] += dir_y * force_magnitude

        return total_force

    def calculate_attractive_force(self, current_pose, goal):
        # Calculate attractive force toward goal
        dx = goal.pose.position.x - current_pose.pose.position.x
        dy = goal.pose.position.y - current_pose.pose.position.y
        distance = math.sqrt(dx*dx + dy*dy)

        if distance < 0.1:  # Close to goal
            return np.array([0.0, 0.0])

        # Normalize direction toward goal
        dir_x = dx / distance
        dir_y = dy / distance

        # Calculate force (linear attraction)
        force_magnitude = min(distance * 0.5, 2.0)  # Cap maximum force
        return np.array([dir_x * force_magnitude, dir_y * force_magnitude])

    def apply_humanoid_constraints(self, velocity):
        # Apply humanoid-specific movement constraints
        speed = math.sqrt(velocity[0]**2 + velocity[1]**2)

        # Limit maximum speed based on humanoid capabilities
        max_speed = 0.5  # m/s for humanoid walking
        if speed > max_speed:
            velocity = [v * max_speed / speed for v in velocity]

        # Limit turning rate
        if abs(velocity[1]) > 0.3:  # Sideways movement constraint
            velocity[1] = 0.3 if velocity[1] > 0 else -0.3

        return velocity
```

### Trajectory Control for Humanoid Locomotion

Controlling humanoid trajectories requires special consideration:

```python
# Example: Humanoid trajectory controller
from geometry_msgs.msg import Twist, Pose
from nav_msgs.msg import Path
import math

class HumanoidTrajectoryController:
    def __init__(self, node):
        self.node = node
        self.kp_linear = 1.5  # Proportional gain for linear velocity
        self.ki_linear = 0.1  # Integral gain for linear velocity
        self.kd_linear = 0.2  # Derivative gain for linear velocity

        self.kp_angular = 2.0  # Proportional gain for angular velocity
        self.ki_angular = 0.1  # Integral gain for angular velocity
        self.kd_angular = 0.3  # Derivative gain for angular velocity

        self.error_integral_linear = 0.0
        self.prev_error_linear = 0.0

        self.error_integral_angular = 0.0
        self.prev_error_angular = 0.0

        # Humanoid-specific parameters
        self.max_linear_speed = 0.5  # m/s
        self.max_angular_speed = 0.5  # rad/s
        self.min_approach_dist = 0.3  # Minimum distance before slowing down

    def compute_velocity_command(self, current_pose, target_pose, path_progress=0.0):
        # Compute velocity command for humanoid navigation
        # Calculate distance and bearing to target
        dx = target_pose.pose.position.x - current_pose.pose.position.x
        dy = target_pose.pose.position.y - current_pose.pose.position.y
        distance = math.sqrt(dx*dx + dy*dy)

        # Calculate desired heading
        desired_heading = math.atan2(dy, dx)

        # Get current heading from orientation
        current_heading = self.quaternion_to_yaw(current_pose.pose.orientation)

        # Calculate heading error
        heading_error = self.normalize_angle(desired_heading - current_heading)

        # PID control for linear velocity
        linear_error = distance

        # Slow down when approaching target
        deceleration_factor = min(1.0, distance / self.min_approach_dist)

        self.error_integral_linear += linear_error * 0.1  # Assuming 10Hz control
        linear_derivative = (linear_error - self.prev_error_linear) / 0.1

        linear_velocity = (
            self.kp_linear * linear_error * deceleration_factor +
            self.ki_linear * self.error_integral_linear +
            self.kd_linear * linear_derivative
        )

        # Clamp linear velocity
        linear_velocity = max(0.0, min(linear_velocity, self.max_linear_speed))

        # PID control for angular velocity
        self.error_integral_angular += heading_error * 0.1
        angular_derivative = (heading_error - self.prev_error_angular) / 0.1

        angular_velocity = (
            self.kp_angular * heading_error +
            self.ki_angular * self.error_integral_angular +
            self.kd_angular * angular_derivative
        )

        # Clamp angular velocity
        angular_velocity = max(-self.max_angular_speed, min(angular_velocity, self.max_angular_speed))

        # Store current errors for next iteration
        self.prev_error_linear = linear_error
        self.prev_error_angular = heading_error

        # Create velocity command
        cmd_vel = Twist()
        cmd_vel.linear.x = linear_velocity
        cmd_vel.angular.z = angular_velocity

        return cmd_vel

    def quaternion_to_yaw(self, quat):
        # Convert quaternion to yaw angle
        siny_cosp = 2 * (quat.w * quat.z + quat.x * quat.y)
        cosy_cosp = 1 - 2 * (quat.y * quat.y + quat.z * quat.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def normalize_angle(self, angle):
        # Normalize angle to [-pi, pi]
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle
```

## Preparing for Autonomous Tasks with Safety Measures

### Humanoid Navigation Safety Framework

```python
# Example: Safety framework for humanoid navigation
from enum import Enum
import threading

class SafetyLevel(Enum):
    OPERATIONAL = 1
    CAUTION = 2
    WARNING = 3
    EMERGENCY_STOP = 4

class HumanoidNavigationSafety:
    def __init__(self, node):
        self.node = node
        self.safety_level = SafetyLevel.OPERATIONAL
        self.emergency_stop_requested = False
        self.safety_lock = threading.Lock()

        # Safety parameters
        self.critical_distance = 0.2  # Distance for emergency stop
        self.warning_distance = 0.5   # Distance for warning
        self.caution_distance = 1.0   # Distance for caution
        self.max_fall_risk_tilt = 0.3 # Max tilt before fall risk

    def assess_environment_safety(self, obstacles, terrain_map, robot_state):
        # Assess safety of current environment
        with self.safety_lock:
            # Check for immediate collision risks
            immediate_risks = self.check_immediate_dangers(obstacles, robot_state)

            if immediate_risks:
                self.safety_level = SafetyLevel.EMERGENCY_STOP
                self.emergency_stop_requested = True
                return self.safety_level

            # Check for potential hazards
            potential_hazards = self.check_potential_hazards(terrain_map, obstacles)

            if potential_hazards:
                self.safety_level = SafetyLevel.WARNING
            elif self.is_complex_terrain(terrain_map):
                self.safety_level = SafetyLevel.CAUTION
            else:
                self.safety_level = SafetyLevel.OPERATIONAL

        return self.safety_level

    def check_immediate_dangers(self, obstacles, robot_state):
        # Check for immediate collision dangers
        for obstacle in obstacles:
            dist = self.distance_to_obstacle(obstacle, robot_state)
            if dist < self.critical_distance:
                return True
        return False

    def check_potential_hazards(self, terrain_map, obstacles):
        # Check for potential navigation hazards
        for obstacle in obstacles:
            dist = self.distance_to_obstacle(obstacle, robot_state)
            if dist < self.warning_distance:
                return True
        return False

    def request_emergency_stop(self):
        # Request immediate stop for safety
        with self.safety_lock:
            self.emergency_stop_requested = True
            self.safety_level = SafetyLevel.EMERGENCY_STOP

    def clear_emergency_stop(self):
        # Clear emergency stop condition
        with self.safety_lock:
            self.emergency_stop_requested = False
            # Reset to operational after assessment
            self.safety_level = SafetyLevel.OPERATIONAL
```

### Autonomous Task Preparation

```python
# Example: Preparing humanoid for autonomous tasks
class HumanoidAutonomousTaskManager:
    def __init__(self, node):
        self.node = node
        self.navigation_safety = HumanoidNavigationSafety(node)
        self.task_queue = []
        self.current_task = None

    def prepare_for_autonomous_navigation(self, goal_pose):
        # Prepare humanoid robot for autonomous navigation task
        self.node.get_logger().info("Preparing for autonomous navigation...")

        # 1. Validate goal position
        if not self.validate_goal_position(goal_pose):
            self.node.get_logger().error("Invalid goal position")
            return False

        # 2. Assess current safety state
        current_safety = self.navigation_safety.assess_environment_safety(
            self.get_current_obstacles(),
            self.get_current_terrain(),
            self.get_robot_state()
        )

        if current_safety == SafetyLevel.EMERGENCY_STOP:
            self.node.get_logger().error("Environment unsafe for navigation")
            return False

        # 3. Plan initial path
        path = self.plan_initial_path(goal_pose)
        if path is None:
            self.node.get_logger().error("Could not plan initial path")
            return False

        # 4. Check path safety
        if not self.is_path_safe(path):
            self.node.get_logger().error("Planned path is not safe")
            return False

        # 5. Configure controllers
        self.configure_controllers_for_humanoid()

        # 6. Set up safety monitors
        self.setup_safety_monitors()

        # 7. Begin navigation
        self.begin_navigation(goal_pose)

        return True

    def validate_goal_position(self, goal_pose):
        # Validate that goal position is reachable and safe
        # Check if goal is on traversable terrain
        # Check if goal is within operational bounds
        # Check if goal is not in restricted area
        return True  # Simplified implementation

    def is_path_safe(self, path):
        # Check if the planned path is safe for humanoid navigation
        for waypoint in path.poses:
            # Check terrain traversability at waypoint
            terrain_safe = self.is_terrain_safe_at(waypoint.pose.position)
            if not terrain_safe:
                return False

            # Check for potential hazards along path
            if self.has_hazards_near_path(waypoint.pose.position):
                return False

        return True

    def setup_safety_monitors(self):
        # Set up continuous safety monitoring during navigation
        self.safety_timer = self.node.create_timer(
            0.1,  # Check every 100ms
            self.safety_monitor_callback
        )

    def safety_monitor_callback(self):
        # Continuously monitor safety during navigation
        current_state = self.get_robot_state()
        obstacles = self.get_current_obstacles()
        terrain = self.get_current_terrain()

        safety_level = self.navigation_safety.assess_environment_safety(
            obstacles, terrain, current_state
        )

        if safety_level == SafetyLevel.EMERGENCY_STOP:
            self.execute_emergency_stop()
        elif safety_level == SafetyLevel.WARNING:
            self.reduce_navigation_speed()
        elif safety_level == SafetyLevel.CAUTION:
            self.increase_caution()
```

## Practical Exercise: Complete Navigation System

Let's create a complete example that ties all concepts together:

```python
# Complete example: Humanoid Navigation System
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped, Point
from nav_msgs.msg import Path, OccupancyGrid
from sensor_msgs.msg import LaserScan, Image
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import math
import numpy as np

class CompleteHumanoidNavigationSystem(Node):
    def __init__(self):
        super().__init__('complete_humanoid_navigation_system')

        # Initialize components
        self.planner = HumanoidPathPlanner(self)
        self.controller = HumanoidTrajectoryController(self)
        self.obstacle_avoider = HumanoidObstacleAvoider(self)
        self.safety_manager = HumanoidNavigationSafety(self)
        self.task_manager = HumanoidAutonomousTaskManager(self)

        # TF for pose transformations
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Publishers and subscribers
        self.cmd_vel_pub = self.create_publisher(Twist, '/humanoid/cmd_vel', 10)
        self.path_pub = self.create_publisher(Path, '/humanoid/current_path', 10)
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10
        )
        self.goal_sub = self.create_subscription(
            PoseStamped, '/move_base_simple/goal', self.goal_callback, 10
        )

        # Navigation state
        self.current_goal = None
        self.navigation_active = False
        self.current_path = None
        self.last_command_time = self.get_clock().now()

        # Control timer
        self.control_timer = self.create_timer(0.1, self.navigation_control_loop)  # 10Hz

        self.get_logger().info("Complete Humanoid Navigation System initialized")

    def scan_callback(self, msg):
        # Process laser scan for obstacle detection
        self.last_scan = msg
        # Update obstacle avoider with new scan data
        self.detect_obstacles_from_scan(msg)

    def goal_callback(self, msg):
        # Handle new navigation goal
        self.get_logger().info(f"Received navigation goal: {msg.pose.position}")
        self.set_navigation_goal(msg.pose)

    def set_navigation_goal(self, goal_pose):
        # Set new navigation goal and start planning
        self.current_goal = goal_pose
        self.navigation_active = True

        # Prepare for navigation
        success = self.task_manager.prepare_for_autonomous_navigation(goal_pose)
        if success:
            self.get_logger().info("Navigation prepared successfully")
        else:
            self.get_logger().error("Failed to prepare for navigation")
            self.navigation_active = False

    def navigation_control_loop(self):
        # Main navigation control loop
        if not self.navigation_active or self.current_goal is None:
            return

        try:
            # Get current robot pose
            current_pose = self.get_current_pose()
            if current_pose is None:
                self.get_logger().warn("Could not get current robot pose")
                return

            # Check if goal reached
            distance_to_goal = self.distance_between_poses(current_pose.pose, self.current_goal)
            if distance_to_goal < 0.5:  # 50cm tolerance
                self.get_logger().info("Goal reached!")
                self.navigation_active = False
                self.current_goal = None
                return

            # Assess safety
            obstacles = self.get_current_obstacles()
            terrain_map = self.get_current_terrain()
            robot_state = self.get_robot_state()

            safety_level = self.safety_manager.assess_environment_safety(
                obstacles, terrain_map, robot_state
            )

            if safety_level == self.safety_manager.SafetyLevel.EMERGENCY_STOP:
                self.emergency_stop()
                return

            # Compute navigation command
            cmd_vel = self.compute_navigation_command(current_pose.pose, self.current_goal)

            # Publish command
            self.cmd_vel_pub.publish(cmd_vel)
            self.last_command_time = self.get_clock().now()

        except Exception as e:
            self.get_logger().error(f"Error in navigation control loop: {e}")

    def compute_navigation_command(self, current_pose, goal_pose):
        # Compute the appropriate velocity command
        # First, try path following
        if self.current_path and self.is_path_valid():
            # Follow current path
            next_waypoint = self.get_next_waypoint_along_path(current_pose)
            if next_waypoint:
                cmd_vel = self.controller.compute_velocity_command(
                    current_pose, next_waypoint
                )
            else:
                # Path is complete, head to goal
                cmd_vel = self.controller.compute_velocity_command(
                    current_pose, goal_pose
                )
        else:
            # Plan new path or use direct control
            if self.should_replan_path(current_pose, goal_pose):
                self.current_path = self.planner.plan_path(current_pose, goal_pose)

            # Use direct control toward goal
            cmd_vel = self.controller.compute_velocity_command(
                current_pose, goal_pose
            )

        # Apply obstacle avoidance if needed
        if self.has_close_obstacles():
            avoidance_cmd = self.obstacle_avoider.compute_avoidance_velocity(
                current_pose, cmd_vel, goal_pose
            )
            # Blend path-following and obstacle avoidance commands
            cmd_vel = self.blend_commands(cmd_vel, avoidance_cmd)

        return cmd_vel

    def get_current_pose(self):
        # Get current robot pose from TF
        try:
            transform = self.tf_buffer.lookup_transform(
                'map', 'base_link', rclpy.time.Time(), rclpy.duration.Duration(seconds=1.0)
            )

            pose_stamped = PoseStamped()
            pose_stamped.header.frame_id = 'map'
            pose_stamped.header.stamp = transform.header.stamp

            pose_stamped.pose.position.x = transform.transform.translation.x
            pose_stamped.pose.position.y = transform.transform.translation.y
            pose_stamped.pose.position.z = transform.transform.translation.z
            pose_stamped.pose.orientation = transform.transform.rotation

            return pose_stamped
        except TransformException as ex:
            self.get_logger().error(f'Could not transform map to base_link: {ex}')
            return None

    def blend_commands(self, path_cmd, avoid_cmd):
        # Blend path-following and obstacle avoidance commands
        blended_cmd = Twist()

        # Weight avoidance more heavily when obstacles are very close
        avoidance_weight = self.get_obstacle_proximity_weight()

        blended_cmd.linear.x = (
            (1 - avoidance_weight) * path_cmd.linear.x +
            avoidance_weight * avoid_cmd.linear.x
        )

        blended_cmd.angular.z = (
            (1 - avoidance_weight) * path_cmd.angular.z +
            avoidance_weight * avoid_cmd.angular.z
        )

        return blended_cmd

    def emergency_stop(self):
        # Execute emergency stop
        stop_cmd = Twist()
        self.cmd_vel_pub.publish(stop_cmd)
        self.navigation_active = False
        self.get_logger().warn("Emergency stop executed!")

def main(args=None):
    rclpy.init(args=args)

    node = CompleteHumanoidNavigationSystem()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down navigation system...")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Summary

In this chapter, you've learned how to:
- Use Nav2 specifically adapted for bipedal humanoid movement patterns and constraints
- Implement various motion planning algorithms suitable for humanoid robots
- Apply obstacle avoidance and trajectory control techniques for safe humanoid navigation
- Prepare humanoid robots for autonomous tasks with proper safety measures
- Create a complete navigation system integrating all concepts

You now have a complete understanding of the AI-Robot Brain (NVIDIA Isaac™) module, covering Isaac Sim for simulation, Isaac ROS for perception and navigation, and Nav2 for path planning. This foundation prepares you for more advanced topics in vision-language-action control in Module 4.

## Navigation

- Previous: [Chapter 2 - Isaac ROS for Perception and Navigation](./02-isaac-ros-perception-navigation)
- Back to: [Module Overview](../)