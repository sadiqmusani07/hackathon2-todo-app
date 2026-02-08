---
sidebar_position: 2
title: "Module 4, Chapter 2: Cognitive Planning with Large Language Models"
description: "Learn to implement cognitive planning using LLMs to translate natural language goals into action plans for humanoid robots"
---

# Module 4, Chapter 2: Cognitive Planning with Large Language Models

## Overview

Large Language Models (LLMs) represent a breakthrough in artificial intelligence that enables natural language understanding and reasoning capabilities. In this chapter, we'll explore how to implement cognitive planning for humanoid robots using LLMs to translate natural language goals into executable action plans. We'll cover the role of LLMs in robotic reasoning, safety considerations, and integration with ROS 2 for actionable robot commands.

## Role of LLMs in Robotic Reasoning

### Cognitive Architecture for Humanoid Robots

LLMs serve as the cognitive layer in humanoid robots, bridging the gap between high-level natural language commands and low-level motor actions. This cognitive layer handles:

- **Natural Language Understanding**: Interpreting human commands and requests
- **Task Decomposition**: Breaking complex goals into executable subtasks
- **Knowledge Integration**: Incorporating world knowledge and commonsense reasoning
- **Plan Synthesis**: Creating action sequences that achieve desired goals
- **Context Awareness**: Understanding situational context and environmental constraints


### LLM Integration Patterns

```python
# Example: LLM integration for robotic reasoning
import openai
import json
import asyncio
from typing import Dict, List, Optional
from dataclasses import dataclass

@dataclass
class ActionPlan:
    """Structured representation of robot action plan"""
    id: str
    goal: str
    steps: List[Dict]
    constraints: List[str]
    estimated_duration: float

class LLMBasedCognitivePlanner:
    def __init__(self, api_key: str, model: str = "gpt-4"):
        self.client = openai.OpenAI(api_key=api_key)
        self.model = model
        self.robot_capabilities = self._load_robot_capabilities()

    def _load_robot_capabilities(self) -> Dict:
        """Load robot-specific capabilities and constraints"""
        return {
            "locomotion": {
                "max_speed": 0.5,  # m/s
                "step_height": 0.1,  # m
                "turning_radius": 0.3,  # m
                "balance_constraints": True
            },
            "manipulation": {
                "reach": 1.2,  # m
                "grip_force": 50,  # N
                "precision_modes": ["fine", "coarse", "power"]
            },
            "perception": {
                "lidar_range": 10.0,  # m
                "camera_resolution": [640, 480],
                "detection_range": 5.0  # m
            },
            "navigation": {
                "obstacle_detection": True,
                "path_planning": True,
                "localization_accuracy": 0.05  # m
            }
        }

    def generate_action_plan(self, goal: str, context: Optional[Dict] = None) -> ActionPlan:
        """Generate action plan from natural language goal using LLM"""

        # Construct system prompt with robot capabilities
        system_prompt = f"""
        You are a cognitive planning system for a humanoid robot. Your role is to:
        1. Interpret natural language goals
        2. Decompose complex goals into executable subtasks
        3. Consider the robot's physical capabilities and limitations
        4. Generate safe and feasible action plans
        5. Include error handling and recovery strategies

        Robot capabilities:
        - Locomotion: Max speed {self.robot_capabilities['locomotion']['max_speed']}m/s, step height {self.robot_capabilities['locomotion']['step_height']}m
        - Manipulation: Reach {self.robot_capabilities['manipulation']['reach']}m, grip force {self.robot_capabilities['manipulation']['grip_force']}N
        - Perception: LiDAR range {self.robot_capabilities['perception']['lidar_range']}m, camera resolution {self.robot_capabilities['perception']['camera_resolution']}
        - Navigation: Obstacle detection, path planning, localization accuracy {self.robot_capabilities['navigation']['localization_accuracy']}m

        Constraints:
        - Always prioritize safety
        - Consider balance and stability for humanoid movement
        - Respect physical limitations of the robot
        - Handle ambiguous or impossible requests gracefully

        Respond with a JSON object containing:
        {{
            "id": "unique_plan_id",
            "goal": "original goal text",
            "steps": [
                {{
                    "id": "step_id",
                    "action": "action_type",
                    "parameters": {{"param1": "value1", ...}},
                    "expected_outcome": "description of expected outcome",
                    "safety_check": "safety validation to perform",
                    "recovery_plan": "what to do if step fails"
                }}
            ],
            "constraints": ["list of safety constraints"],
            "estimated_duration": "estimated time in seconds"
        }}
        """

        # Construct user message with goal and context
        user_message = f"Goal: {goal}\n\nContext: {json.dumps(context or {})}"

        try:
            response = self.client.chat.completions.create(
                model=self.model,
                messages=[
                    {"role": "system", "content": system_prompt},
                    {"role": "user", "content": user_message}
                ],
                temperature=0.1,  # Low temperature for consistency
                max_tokens=2048,
                response_format={"type": "json_object"}
            )

            # Parse response
            plan_data = json.loads(response.choices[0].message.content)
            return ActionPlan(**plan_data)

        except Exception as e:
            raise RuntimeError(f"Failed to generate action plan: {e}")

    def validate_plan_safety(self, plan: ActionPlan) -> tuple[bool, List[str]]:
        """Validate plan for safety and feasibility"""
        issues = []

        # Check each step for safety
        for step in plan.steps:
            action = step.get('action', '')
            params = step.get('parameters', {})

            # Validate locomotion safety
            if action in ['move', 'navigate', 'walk']:
                if params.get('speed', 0) > self.robot_capabilities['locomotion']['max_speed']:
                    issues.append(f"Step {step['id']}: Movement speed exceeds maximum capability")

            # Validate manipulation safety
            elif action in ['grasp', 'manipulate', 'lift']:
                if params.get('force', 0) > self.robot_capabilities['manipulation']['grip_force']:
                    issues.append(f"Step {step['id']}: Grip force exceeds maximum capability")

        return len(issues) == 0, issues
```

### Safety and Grounding Considerations

LLM-driven robot control requires careful safety measures:

```python
# Example: Safety and grounding for LLM-based robot control
class SafetyGroundingLayer:
    def __init__(self):
        self.forbidden_actions = [
            'jump', 'fly', 'break', 'damage', 'hurt', 'destroy',
            'self_harm', 'harm_others', 'violate_privacy'
        ]

        self.physical_constraints = {
            'max_height': 2.0,  # Robot cannot reach above this height
            'min_distance': 0.1,  # Minimum safe distance from obstacles
            'max_force': 100.0,   # Maximum force for any action
            'max_velocity': 1.0   # Maximum velocity for any movement
        }

        self.ethical_constraints = [
            "respect_persons",
            "maintain_privacy",
            "follow_social_norms",
            "avoid_harm"
        ]

    def ground_llm_output(self, llm_output: Dict) -> tuple[bool, str, Dict]:
        """Apply safety and grounding constraints to LLM output"""

        # Check for forbidden actions
        for step in llm_output.get('steps', []):
            action = step.get('action', '').lower()

            if action in self.forbidden_actions:
                return False, f"Forbidden action detected: {action}", {}

            # Validate physical constraints
            params = step.get('parameters', {})

            if 'height' in params and params['height'] > self.physical_constraints['max_height']:
                return False, f"Height constraint violation: {params['height']}m", {}

            if 'distance' in params and params['distance'] < self.physical_constraints['min_distance']:
                return False, f"Distance constraint violation: {params['distance']}m", {}

            if 'force' in params and params['force'] > self.physical_constraints['max_force']:
                return False, f"Force constraint violation: {params['force']}N", {}

            if 'velocity' in params and params['velocity'] > self.physical_constraints['max_velocity']:
                return False, f"Velocity constraint violation: {params['velocity']}m/s", {}

        # Apply ethical constraints
        grounded_output = self._apply_ethical_grounding(llm_output)

        return True, "Output is safe and grounded", grounded_output

    def _apply_ethical_grounding(self, output: Dict) -> Dict:
        """Apply ethical grounding to LLM output"""
        # Remove any actions that violate ethical constraints
        filtered_steps = []

        for step in output.get('steps', []):
            action_desc = f"{step.get('action', '')} {step.get('parameters', '')}".lower()

            # Check if action respects persons
            if any(person_term in action_desc for person_term in ['person', 'human', 'individual']):
                # Add safety checks for human interaction
                if 'approach' in action_desc or 'move_to' in action_desc:
                    step.setdefault('safety_checks', []).append('maintain_safe_distance')

            # Check if action maintains privacy
            if any(privacy_term in action_desc for privacy_term in ['record', 'capture', 'take_photo', 'observe_private']):
                step.setdefault('safety_checks', []).append('request_consent')

            filtered_steps.append(step)

        output['steps'] = filtered_steps
        return output
```

## Translating Natural Language Goals to Action Plans

### Natural Language Understanding Pipeline

The process of translating natural language to action plans involves several steps:

```python
# Example: Natural language to action plan translation
import re
from typing import Dict, Any

class NaturalLanguageToAction:
    def __init__(self):
        # Define action mappings
        self.action_mappings = {
            'move': ['move', 'go', 'walk', 'step', 'travel', 'navigate'],
            'rotate': ['turn', 'rotate', 'spin', 'pivot', 'face'],
            'manipulate': ['pick', 'grasp', 'grab', 'hold', 'lift', 'place', 'put', 'release'],
            'perceive': ['look', 'see', 'find', 'locate', 'detect', 'observe', 'examine'],
            'communicate': ['say', 'speak', 'tell', 'announce', 'report', 'answer'],
            'navigate': ['goto', 'go_to', 'navigate_to', 'reach', 'arrive_at']
        }

        # Location keywords
        self.location_keywords = [
            'kitchen', 'bedroom', 'living room', 'office', 'bathroom',
            'table', 'chair', 'door', 'window', 'couch', 'desk'
        ]

        # Object keywords
        self.object_keywords = [
            'cup', 'bottle', 'book', 'phone', 'computer', 'plate',
            'fork', 'knife', 'spoon', 'box', 'ball', 'toy'
        ]

    def parse_goal(self, goal_text: str) -> Dict[str, Any]:
        """Parse natural language goal into structured format"""
        goal_text_lower = goal_text.lower()

        # Extract action
        action = self._extract_action(goal_text_lower)

        # Extract target/object
        target = self._extract_target(goal_text_lower)

        # Extract location
        location = self._extract_location(goal_text_lower)

        # Extract parameters (quantities, directions, etc.)
        parameters = self._extract_parameters(goal_text_lower)

        return {
            'action': action,
            'target': target,
            'location': location,
            'parameters': parameters,
            'original_text': goal_text
        }

    def _extract_action(self, text: str) -> str:
        """Extract primary action from text"""
        for action_type, keywords in self.action_mappings.items():
            for keyword in keywords:
                if keyword in text:
                    return action_type
        return 'unknown'

    def _extract_target(self, text: str) -> str:
        """Extract target object from text"""
        for obj in self.object_keywords:
            if obj in text:
                return obj
        return None

    def _extract_location(self, text: str) -> str:
        """Extract location from text"""
        for loc in self.location_keywords:
            if loc in text.replace('_', ' '):
                return loc
        return None

    def _extract_parameters(self, text: str) -> Dict[str, Any]:
        """Extract numerical and spatial parameters"""
        params = {}

        # Extract distances (numbers followed by distance units)
        distance_pattern = r'(\d+(?:\.\d+)?)\s*(m|cm|mm|meter|feet|ft)'
        distances = re.findall(distance_pattern, text)
        if distances:
            params['distance'] = float(distances[0][0])
            params['unit'] = distances[0][1]

        # Extract directions
        directions = ['forward', 'backward', 'left', 'right', 'up', 'down']
        for direction in directions:
            if direction in text:
                params['direction'] = direction

        # Extract speeds
        speed_pattern = r'(\d+(?:\.\d+)?)\s*(m/s|cm/s|mph|kph)'
        speeds = re.findall(speed_pattern, text)
        if speeds:
            params['speed'] = float(speeds[0][0])
            params['speed_unit'] = speeds[0][1]

        return params

    def convert_to_action_plan(self, parsed_goal: Dict) -> ActionPlan:
        """Convert parsed goal to executable action plan"""

        # Create action steps based on parsed goal
        steps = []

        if parsed_goal['action'] == 'navigate':
            steps.extend(self._create_navigation_steps(parsed_goal))
        elif parsed_goal['action'] == 'manipulate':
            steps.extend(self._create_manipulation_steps(parsed_goal))
        elif parsed_goal['action'] == 'perceive':
            steps.extend(self._create_perception_steps(parsed_goal))
        else:
            steps.extend(self._create_generic_steps(parsed_goal))

        return ActionPlan(
            id=f"plan_{hash(str(parsed_goal)) % 10000}",
            goal=parsed_goal['original_text'],
            steps=steps,
            constraints=[],
            estimated_duration=len(steps) * 2.0  # 2 seconds per step estimate
        )

    def _create_navigation_steps(self, parsed_goal: Dict) -> List[Dict]:
        """Create navigation-specific action steps"""
        steps = []

        # Look for target location in goal
        if parsed_goal.get('location'):
            steps.append({
                'id': 'nav_find_target',
                'action': 'locate',
                'parameters': {'target': parsed_goal['location']},
                'expected_outcome': f'Location {parsed_goal["location"]} identified',
                'safety_check': 'validate_location_accessibility',
                'recovery_plan': 'find_alternative_route'
            })

            steps.append({
                'id': 'nav_move_to_target',
                'action': 'navigate',
                'parameters': {
                    'destination': parsed_goal['location'],
                    'speed': 0.3,
                    'avoid_obstacles': True
                },
                'expected_outcome': f'Reached {parsed_goal["location"]}',
                'safety_check': 'maintain_balance',
                'recovery_plan': 'stop_and_replan'
            })

        return steps

    def _create_manipulation_steps(self, parsed_goal: Dict) -> List[Dict]:
        """Create manipulation-specific action steps"""
        steps = []

        if parsed_goal.get('target'):
            # First locate the object
            steps.append({
                'id': 'manip_locate_object',
                'action': 'locate',
                'parameters': {'target': parsed_goal['target']},
                'expected_outcome': f'{parsed_goal["target"]} located',
                'safety_check': 'validate_object_graspability',
                'recovery_plan': 'abort_manipulation'
            })

            # Then approach the object
            steps.append({
                'id': 'manip_approach_object',
                'action': 'move',
                'parameters': {
                    'target': parsed_goal['target'],
                    'distance': 0.3,  # 30cm from object
                    'speed': 0.1
                },
                'expected_outcome': f'Close to {parsed_goal["target"]}',
                'safety_check': 'validate_approach_safety',
                'recovery_plan': 'stop_approach'
            })

            # Finally manipulate the object
            steps.append({
                'id': 'manip_execute',
                'action': 'manipulate',
                'parameters': {
                    'target': parsed_goal['target'],
                    'operation': 'grasp'  # Default operation
                },
                'expected_outcome': f'{parsed_goal["target"]} manipulated',
                'safety_check': 'validate_manipulation_safety',
                'recovery_plan': 'release_and_retreat'
            })

        return steps
```

### Context-Aware Planning

Humanoid robots need to consider context when planning:

```python
# Example: Context-aware planning for humanoid robots
class ContextAwarePlanner:
    def __init__(self):
        self.context_memory = {}
        self.environment_map = {}  # Current environment state
        self.robot_state = {}      # Current robot state
        self.social_context = {}   # Social situation awareness

    def plan_with_context(self, goal: str, current_context: Dict) -> ActionPlan:
        """Generate action plan considering current context"""

        # Update context memory
        self._update_context_memory(current_context)

        # Enhance goal with contextual information
        enhanced_goal = self._enhance_goal_with_context(goal, current_context)

        # Generate plan with context-aware LLM prompting
        plan = self._generate_contextual_plan(enhanced_goal)

        # Apply context-specific constraints
        constrained_plan = self._apply_context_constraints(plan, current_context)

        return constrained_plan

    def _update_context_memory(self, context: Dict):
        """Update context memory with new information"""
        self.context_memory.update(context)

        # Update environment map
        if 'objects' in context:
            self.environment_map.update(context['objects'])

        # Update robot state
        if 'robot_state' in context:
            self.robot_state.update(context['robot_state'])

        # Update social context
        if 'social' in context:
            self.social_context.update(context['social'])

    def _enhance_goal_with_context(self, goal: str, context: Dict) -> str:
        """Enhance goal with contextual information"""
        enhanced_parts = [goal]

        # Add environment context
        if 'location' in context:
            enhanced_parts.append(f"Current location: {context['location']}")

        if 'time_of_day' in context:
            enhanced_parts.append(f"Time of day: {context['time_of_day']}")

        if 'people_present' in context:
            people = ', '.join(context['people_present'])
            enhanced_parts.append(f"People present: {people}")

        # Add robot state context
        if 'battery_level' in context:
            enhanced_parts.append(f"Current battery level: {context['battery_level']}%")

        if 'current_pose' in context:
            pose = context['current_pose']
            enhanced_parts.append(f"Current pose: x={pose['x']}, y={pose['y']}, theta={pose['theta']}")

        return '\n'.join(enhanced_parts)

    def _generate_contextual_plan(self, enhanced_goal: str) -> ActionPlan:
        """Generate plan considering context using LLM"""
        # This would call the LLM with enhanced context
        # For brevity, returning a simplified plan
        return self._simple_contextual_plan(enhanced_goal)

    def _apply_context_constraints(self, plan: ActionPlan, context: Dict) -> ActionPlan:
        """Apply context-specific constraints to the plan"""

        # Modify plan based on battery level
        if context.get('battery_level', 100) < 20:
            plan.constraints.append("Conserve energy: avoid unnecessary movements")
            plan.constraints.append("Prioritize charging station navigation if possible")

        # Modify plan based on time of day
        if context.get('time_of_day') == 'night':
            plan.constraints.append("Reduce movement speed for safety")
            plan.constraints.append("Use navigation lights if available")

        # Modify plan based on people presence
        if context.get('people_present'):
            plan.constraints.append("Maintain safe distance from humans")
            plan.constraints.append("Announce movements before executing")

        # Add social constraints
        if context.get('social_context', {}).get('formal_setting', False):
            plan.constraints.append("Use formal communication")
            plan.constraints.append("Maintain professional posture")

        return plan

    def _simple_contextual_plan(self, enhanced_goal: str) -> ActionPlan:
        """Simple implementation for demonstration"""
        # In a real implementation, this would call the LLM
        # For this example, we'll return a basic plan
        return ActionPlan(
            id="contextual_plan_001",
            goal=enhanced_goal,
            steps=[
                {
                    'id': 'analyze_context',
                    'action': 'analyze',
                    'parameters': {'context': enhanced_goal},
                    'expected_outcome': 'Context analyzed and understood',
                    'safety_check': 'none',
                    'recovery_plan': 'ask_for_clarification'
                },
                {
                    'id': 'execute_action',
                    'action': 'execute',
                    'parameters': {'goal': enhanced_goal},
                    'expected_outcome': 'Goal achieved considering context',
                    'safety_check': 'apply_contextual_safety',
                    'recovery_plan': 'reassess_and_retry'
                }
            ],
            constraints=[],
            estimated_duration=10.0
        )
```

## Mapping Symbolic Plans to ROS 2 Actions

### ROS 2 Action Integration

Convert symbolic plans to executable ROS 2 actions:

```python
# Example: Mapping symbolic plans to ROS 2 actions
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Path
from std_msgs.msg import String
from move_base_msgs.action import MoveBase
from sensor_msgs.msg import JointState
from control_msgs.action import FollowJointTrajectory

class PlanToROS2Mapper(Node):
    def __init__(self):
        super().__init__('plan_to_ros2_mapper')

        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.goal_pub = self.create_publisher(PoseStamped, '/move_base_simple/goal', 10)
        self.joint_cmd_pub = self.create_publisher(JointState, '/joint_commands', 10)

        # Action clients
        self.nav_client = ActionClient(self, MoveBase, 'move_base')
        self.manipulation_client = ActionClient(self, FollowJointTrajectory, 'manipulator_controller/follow_joint_trajectory')

        # Plan execution
        self.plan_sub = self.create_subscription(
            String,
            '/action_plan',
            self.plan_callback,
            10
        )

        self.get_logger().info("Plan to ROS 2 Mapper initialized")

    def plan_callback(self, msg):
        """Receive action plan and execute"""
        try:
            # Parse plan from JSON string
            plan_data = json.loads(msg.data)
            plan = ActionPlan(**plan_data)

            # Execute plan steps sequentially
            self.execute_plan(plan)

        except Exception as e:
            self.get_logger().error(f"Error executing plan: {e}")

    def execute_plan(self, plan: ActionPlan):
        """Execute action plan steps"""
        for step in plan.steps:
            success = self.execute_step(step)
            if not success:
                self.get_logger().error(f"Step {step['id']} failed")

                # Execute recovery plan
                recovery_success = self.execute_recovery(step.get('recovery_plan', ''))
                if not recovery_success:
                    self.get_logger().error(f"Recovery plan failed for step {step['id']}")
                    break  # Stop execution if recovery fails

    def execute_step(self, step: Dict) -> bool:
        """Execute individual step based on action type"""
        action_type = step.get('action', 'unknown')

        if action_type == 'navigate':
            return self.execute_navigate_step(step)
        elif action_type == 'move':
            return self.execute_move_step(step)
        elif action_type == 'manipulate':
            return self.execute_manipulate_step(step)
        elif action_type == 'perceive':
            return self.execute_perceive_step(step)
        elif action_type == 'communicate':
            return self.execute_communicate_step(step)
        else:
            self.get_logger().warn(f"Unknown action type: {action_type}")
            return False

    def execute_navigate_step(self, step: Dict) -> bool:
        """Execute navigation step using MoveBase action"""
        try:
            # Wait for action server
            if not self.nav_client.wait_for_server(timeout_sec=5.0):
                self.get_logger().error("Navigation server not available")
                return False

            # Extract destination
            params = step.get('parameters', {})
            destination = params.get('destination')

            if not destination:
                self.get_logger().error("No destination specified for navigation")
                return False

            # Convert destination to PoseStamped
            goal_pose = self._convert_destination_to_pose(destination)

            # Create navigation goal
            goal_msg = MoveBase.Goal()
            goal_msg.target_pose = goal_pose

            # Send goal
            future = self.nav_client.send_goal_async(goal_msg)

            # Wait for result
            rclpy.spin_until_future_complete(self, future)

            result = future.result()
            return result.success if result else False

        except Exception as e:
            self.get_logger().error(f"Navigation step execution failed: {e}")
            return False

    def execute_move_step(self, step: Dict) -> bool:
        """Execute movement step using velocity commands"""
        try:
            params = step.get('parameters', {})

            # Create velocity command
            cmd_vel = Twist()

            if 'direction' in params:
                direction = params['direction']
                speed = params.get('speed', 0.3)

                if direction == 'forward':
                    cmd_vel.linear.x = speed
                elif direction == 'backward':
                    cmd_vel.linear.x = -speed
                elif direction == 'left':
                    cmd_vel.linear.y = speed
                elif direction == 'right':
                    cmd_vel.linear.y = -speed
                elif direction == 'up':
                    cmd_vel.linear.z = speed
                elif direction == 'down':
                    cmd_vel.linear.z = -speed
                elif direction == 'rotate_left':
                    cmd_vel.angular.z = speed
                elif direction == 'rotate_right':
                    cmd_vel.angular.z = -speed

            # Execute for specified duration
            duration = params.get('duration', 1.0)
            self._execute_velocity_command(cmd_vel, duration)

            return True

        except Exception as e:
            self.get_logger().error(f"Move step execution failed: {e}")
            return False

    def execute_manipulate_step(self, step: Dict) -> bool:
        """Execute manipulation step using joint trajectory control"""
        try:
            # Wait for manipulation server
            if not self.manipulation_client.wait_for_server(timeout_sec=5.0):
                self.get_logger().error("Manipulation server not available")
                return False

            params = step.get('parameters', {})
            operation = params.get('operation', 'unknown')

            if operation == 'grasp':
                trajectory = self._create_grasp_trajectory(params)
            elif operation == 'release':
                trajectory = self._create_release_trajectory(params)
            else:
                self.get_logger().error(f"Unsupported manipulation operation: {operation}")
                return False

            # Create manipulation goal
            goal_msg = FollowJointTrajectory.Goal()
            goal_msg.trajectory = trajectory

            # Send goal
            future = self.manipulation_client.send_goal_async(goal_msg)

            # Wait for result
            rclpy.spin_until_future_complete(self, future)

            result = future.result()
            return result.success if result else False

        except Exception as e:
            self.get_logger().error(f"Manipulation step execution failed: {e}")
            return False

    def _execute_velocity_command(self, cmd_vel: Twist, duration: float):
        """Execute velocity command for specified duration"""
        start_time = self.get_clock().now()
        end_time = start_time + Duration(seconds=duration)

        while self.get_clock().now() < end_time and rclpy.ok():
            self.cmd_vel_pub.publish(cmd_vel)
            self.get_clock().sleep_for(Duration(seconds=0.1))  # 10Hz

    def _convert_destination_to_pose(self, destination: str) -> PoseStamped:
        """Convert destination string to PoseStamped"""
        # In a real implementation, this would look up the destination
        # in a map or use semantic navigation
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()

        # Placeholder coordinates - in reality would come from semantic map
        if destination == 'kitchen':
            pose.pose.position.x = 5.0
            pose.pose.position.y = 3.0
        elif destination == 'living_room':
            pose.pose.position.x = 2.0
            pose.pose.position.y = 1.0
        else:
            # Default to current position + offset
            pose.pose.position.x = 1.0
            pose.pose.position.y = 1.0

        return pose

    def execute_recovery(self, recovery_plan: str) -> bool:
        """Execute recovery plan"""
        if recovery_plan == 'stop_and_replan':
            # Stop current execution and signal for replanning
            stop_cmd = Twist()
            self.cmd_vel_pub.publish(stop_cmd)
            return True
        elif recovery_plan == 'abort_and_return_home':
            # Navigate back to home position
            return self._return_to_home()
        else:
            self.get_logger().info(f"No specific recovery for: {recovery_plan}")
            return True  # Continue with best effort

    def _return_to_home(self) -> bool:
        """Navigate back to home position"""
        home_pose = PoseStamped()
        home_pose.header.frame_id = 'map'
        home_pose.pose.position.x = 0.0
        home_pose.pose.position.y = 0.0
        home_pose.pose.position.z = 0.0
        home_pose.pose.orientation.w = 1.0

        self.goal_pub.publish(home_pose)
        return True

def main(args=None):
    rclpy.init(args=args)

    mapper = PlanToROS2Mapper()

    try:
        rclpy.spin(mapper)
    except KeyboardInterrupt:
        pass
    finally:
        mapper.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Safety Validation Layer

Add a safety validation layer to ensure LLM-generated plans are safe:

```python
# Example: Safety validation for LLM-generated plans
class PlanSafetyValidator:
    def __init__(self):
        self.safety_rules = [
            self._validate_navigation_safety,
            self._validate_manipulation_safety,
            self._validate_human_interaction_safety,
            self._validate_energy_consumption,
            self._validate_physical_constraints
        ]

    def validate_plan(self, plan: ActionPlan) -> tuple[bool, List[str], ActionPlan]:
        """Validate plan for safety compliance"""
        issues = []

        # Apply all safety rules
        for rule_func in self.safety_rules:
            is_safe, rule_issues, modified_plan = rule_func(plan)
            issues.extend(rule_issues)
            plan = modified_plan

            if not is_safe:
                return False, issues, plan

        return True, issues, plan

    def _validate_navigation_safety(self, plan: ActionPlan) -> tuple[bool, List[str], ActionPlan]:
        """Validate navigation safety in plan"""
        issues = []

        for step in plan.steps:
            if step.get('action') == 'navigate':
                params = step.get('parameters', {})

                # Check for safe destinations
                dest = params.get('destination', '').lower()
                if dest in ['roof', 'dangerous_area', 'restricted_zone']:
                    issues.append(f"Unsafe destination: {dest}")

                # Check navigation speed
                speed = params.get('speed', 0)
                if speed > 0.8:  # Too fast for humanoid
                    issues.append(f"Navigation speed too high: {speed} m/s")
                    # Modify plan to reduce speed
                    step['parameters']['speed'] = 0.5

        return len(issues) == 0, issues, plan

    def _validate_manipulation_safety(self, plan: ActionPlan) -> tuple[bool, List[str], ActionPlan]:
        """Validate manipulation safety in plan"""
        issues = []

        for step in plan.steps:
            if step.get('action') == 'manipulate':
                params = step.get('parameters', {})

                # Check force limits
                force = params.get('force', 0)
                if force > 50:  # N - too much for delicate manipulation
                    issues.append(f"Force too high for manipulation: {force}N")

                # Check object validity
                target = params.get('target', '').lower()
                if target in ['fragile_item', 'sharp_object']:
                    step.setdefault('safety_checks', []).append('extra_caution_required')

        return len(issues) == 0, issues, plan

    def _validate_human_interaction_safety(self, plan: ActionPlan) -> tuple[bool, List[str], ActionPlan]:
        """Validate human interaction safety in plan"""
        issues = []

        for step in plan.steps:
            action = step.get('action', '').lower()
            params = step.get('parameters', {})

            # Check for actions near humans
            if 'approach' in action or 'move' in action:
                distance = params.get('distance', 1.0)
                if distance < 0.5:  # Too close to humans
                    issues.append(f"Unsafe distance to humans: {distance}m")
                    # Modify to maintain safe distance
                    step['parameters']['distance'] = max(0.5, distance)

        return len(issues) == 0, issues, plan

    def _validate_energy_consumption(self, plan: ActionPlan) -> tuple[bool, List[str], ActionPlan]:
        """Validate energy consumption in plan"""
        # Estimate energy consumption based on plan complexity
        estimated_steps = len(plan.steps)

        if estimated_steps > 50:  # Too many steps for single battery charge
            plan.constraints.append("Energy conservation required: break task into segments")

        return True, [], plan

    def _validate_physical_constraints(self, plan: ActionPlan) -> tuple[bool, List[str], ActionPlan]:
        """Validate physical constraints in plan"""
        issues = []

        for step in plan.steps:
            params = step.get('parameters', {})

            # Check height constraints
            height = params.get('height', 0)
            if height > 1.8:  # Above humanoid reach
                issues.append(f"Height too high: {height}m")

            # Check weight constraints
            weight = params.get('weight', 0)
            if weight > 5:  # 5kg too heavy for humanoid
                issues.append(f"Weight too heavy: {weight}kg")

        return len(issues) == 0, issues, plan
```

## Practical Exercise: Complete LLM-Based Planning System

Let's create a complete example that integrates all concepts:

```python
# Complete example: LLM-based cognitive planning system
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import json
import openai
from typing import Dict, Any

class CompleteLLMCognitiveSystem(Node):
    def __init__(self):
        super().__init__('complete_llm_cognitive_system')

        # Initialize components
        self.planner = LLMBasedCognitivePlanner(api_key=self.get_parameter_or('openai_api_key', ''))
        self.grounding_layer = SafetyGroundingLayer()
        self.plan_validator = PlanSafetyValidator()
        self.ros_mapper = PlanToROS2Mapper(self)

        # Publishers and subscribers
        self.voice_command_sub = self.create_subscription(
            String,
            '/voice_commands',
            self.voice_command_callback,
            10
        )

        self.plan_pub = self.create_publisher(String, '/action_plan', 10)
        self.status_pub = self.create_publisher(String, '/cognitive_status', 10)

        # Context manager
        self.context_manager = ContextAwarePlanner()

        self.get_logger().info("Complete LLM Cognitive System initialized")

    def voice_command_callback(self, msg):
        """Process voice command through full cognitive pipeline"""
        try:
            self.get_logger().info(f"Processing voice command: {msg.data}")

            # 1. Parse natural language goal
            parsed_goal = self.parse_natural_language_goal(msg.data)

            # 2. Get current context
            current_context = self.get_current_context()

            # 3. Create contextual plan
            contextual_plan = self.context_manager.plan_with_context(msg.data, current_context)

            # 4. Validate plan safety
            is_safe, issues, validated_plan = self.plan_validator.validate_plan(contextual_plan)

            if not is_safe:
                self.get_logger().error(f"Plan validation failed: {issues}")
                self.publish_status(f"Plan rejected: {', '.join(issues)}")
                return

            # 5. Apply safety grounding
            is_groundable, grounding_msg, grounded_plan = self.grounding_layer.ground_llm_output(validated_plan.__dict__)

            if not is_groundable:
                self.get_logger().error(f"Plan grounding failed: {grounding_msg}")
                self.publish_status(f"Plan grounded: {grounding_msg}")
                return

            # 6. Publish validated action plan
            plan_json = json.dumps(grounded_plan)
            plan_msg = String()
            plan_msg.data = plan_json
            self.plan_pub.publish(plan_msg)

            self.get_logger().info(f"Published action plan for execution: {len(grounded_plan.get('steps', []))} steps")
            self.publish_status(f"Plan executed: {len(grounded_plan.get('steps', []))} steps")

        except Exception as e:
            self.get_logger().error(f"Error processing voice command: {e}")
            self.publish_status(f"Error: {str(e)}")

    def parse_natural_language_goal(self, text: str) -> Dict[str, Any]:
        """Parse natural language goal using NL processor"""
        nl_processor = NaturalLanguageToAction()
        return nl_processor.parse_goal(text)

    def get_current_context(self) -> Dict:
        """Get current robot and environment context"""
        # In a real implementation, this would gather context from various sensors and systems
        return {
            'location': 'current_room',
            'time_of_day': 'daytime',
            'people_present': [],
            'battery_level': 85,
            'current_pose': {'x': 0.0, 'y': 0.0, 'theta': 0.0},
            'social_context': {'formal_setting': False}
        }

    def publish_status(self, status: str):
        """Publish cognitive system status"""
        status_msg = String()
        status_msg.data = status
        self.status_pub.publish(status_msg)

def main(args=None):
    rclpy.init(args=args)

    cognitive_system = CompleteLLMCognitiveSystem()

    try:
        rclpy.spin(cognitive_system)
    except KeyboardInterrupt:
        cognitive_system.get_logger().info("Shutting down cognitive system...")
    finally:
        cognitive_system.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Summary

In this chapter, you've learned how to:
- Integrate large language models with humanoid robot cognitive systems
- Translate natural language goals into executable action plans
- Apply safety and grounding constraints to LLM outputs
- Map symbolic plans to ROS 2 actions for robot execution
- Implement context-aware planning for adaptive behavior
- Validate plans for safety and feasibility

In the [next chapter](./03-autonomous-humanoid), we'll explore how to build a complete end-to-end system that coordinates all VLA components with ROS 2 for a fully autonomous humanoid robot capable of responding to voice commands with intelligent actions.