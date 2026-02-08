---
sidebar_position: 1
title: "Module 4, Chapter 1: Voice-to-Action with Speech Recognition"
description: "Learn to integrate speech recognition with humanoid robots using OpenAI Whisper and ROS 2 messaging"
---

# Module 4, Chapter 1: Voice-to-Action with Speech Recognition

## Overview

Voice interaction represents a natural and intuitive way for humans to communicate with humanoid robots. This chapter covers how to implement voice-based command systems for humanoid robots using OpenAI Whisper for speech recognition and integrating these capabilities into ROS 2 messaging systems. We'll build a complete pipeline from audio input to actionable commands that can drive humanoid robot behaviors.

## Voice-Based Robot Interaction

### The Importance of Voice Commands

Voice-based interaction is crucial for humanoid robots because it:
- Enables natural human-robot communication without requiring specialized interfaces
- Allows hands-free operation in various environments
- Facilitates collaborative work between humans and robots

### Voice Processing Pipeline Architecture

This architecture includes audio capture, speech recognition with OpenAI Whisper, natural language processing, and ROS 2 command distribution.
- Provides accessibility for users with mobility limitations

### Voice Command Architecture

The voice-to-action pipeline follows this flow:
1. **Audio Capture**: Microphones capture spoken commands
2. **Speech Recognition**: Convert audio to text using Whisper
3. **Command Processing**: Parse and interpret text commands
4. **ROS 2 Integration**: Publish commands to ROS 2 topics for execution

### Design Considerations for Humanoid Robots

When implementing voice recognition for humanoid robots, consider:

- **Environmental Noise**: Humanoid robots often operate in noisy environments
- **Latency Requirements**: Responses should feel natural (under 500ms)
- **Command Ambiguity**: Natural language can be ambiguous
- **Safety Constraints**: Voice commands must be validated before execution
- **Localization**: Support for different languages and accents

## OpenAI Whisper Integration

### Introduction to Whisper

OpenAI Whisper is a state-of-the-art speech recognition system trained on 680,000 hours of multilingual and multitask supervised data. For robotics applications, Whisper provides:

- **High Accuracy**: Robust performance across different accents and environments
- **Multiple Languages**: Support for 99+ languages
- **Real-time Capability**: Can process streaming audio
- **Open Source**: Available under MIT license

### Setting Up Whisper for Robotics

First, install the required dependencies:

```bash
pip install openai-whisper
# Or for GPU acceleration:
pip install openai-whisper[cuda]
```

For robotics applications, we'll use Whisper in a streaming configuration:

```python
import whisper
import torch
import rospy
from audio_common_msgs.msg import AudioData
from std_msgs.msg import String
import numpy as np
import pyaudio
import wave

class WhisperVoiceRecognizer:
    def __init__(self):
        # Initialize Whisper model
        # Use 'tiny' or 'base' for real-time applications, 'large' for accuracy
        self.model = whisper.load_model("base")

        # Initialize PyAudio for audio capture
        self.audio = pyaudio.PyAudio()

        # Audio parameters
        self.FORMAT = pyaudio.paInt16
        self.CHANNELS = 1
        self.RATE = 16000  # Whisper works best at 16kHz
        self.CHUNK = 1024

        # ROS initialization
        rospy.init_node('whisper_voice_recognizer')
        self.command_pub = rospy.Publisher('/voice_commands', String, queue_size=10)

        # Audio stream
        self.stream = self.audio.open(
            format=self.FORMAT,
            channels=self.CHANNELS,
            rate=self.RATE,
            input=True,
            frames_per_buffer=self.CHUNK
        )

        rospy.loginfo("Whisper Voice Recognizer initialized")

    def start_listening(self):
        """Start continuous listening and recognition"""
        rospy.loginfo("Starting voice recognition...")

        while not rospy.is_shutdown():
            # Read audio data
            frames = []

            # Capture a chunk of audio (about 1 second)
            for _ in range(0, int(self.RATE / self.CHUNK * 1)):
                data = self.stream.read(self.CHUNK)
                frames.append(data)

            # Convert to numpy array
            audio_data = np.frombuffer(b''.join(frames), dtype=np.int16)

            # Convert to float32 and normalize
            audio_data = audio_data.astype(np.float32) / 32768.0

            # Run Whisper transcription
            result = self.model.transcribe(audio_data, fp16=torch.cuda.is_available())

            if result["text"].strip():  # Only publish non-empty results
                command_text = result["text"].strip()

                # Publish recognized command
                self.command_pub.publish(command_text)
                rospy.loginfo(f"Recognized: {command_text}")

    def shutdown(self):
        """Clean shutdown of audio resources"""
        self.stream.stop_stream()
        self.stream.close()
        self.audio.terminate()

if __name__ == '__main__':
    recognizer = WhisperVoiceRecognizer()

    try:
        recognizer.start_listening()
    except rospy.ROSInterruptException:
        pass
    finally:
        recognizer.shutdown()
```

### Whisper Model Selection for Robotics

Different Whisper models offer trade-offs between speed and accuracy:

| Model | Size | Speed | Accuracy | Recommended Use Case |
|-------|------|-------|----------|---------------------|
| tiny  | 75MB | Fast  | Lower    | Real-time applications with limited compute |
| base  | 145MB| Fast  | Medium   | Standard robotics applications |
| small | 466MB| Medium| Good     | Higher accuracy requirements |
| medium| 1.5GB| Slow  | Better   | Precision applications |
| large | 3.0GB| Slow  | Best     | Offline processing or high-performance systems |

For humanoid robots, we typically use 'base' or 'small' models to balance accuracy and response time.

### Real-time Processing Techniques

For real-time voice recognition, implement these techniques:

```python
import threading
import queue
import time

class RealTimeWhisperRecognizer:
    def __init__(self):
        self.model = whisper.load_model("base")
        self.audio_queue = queue.Queue()
        self.result_queue = queue.Queue()

        # Audio parameters
        self.FORMAT = pyaudio.paInt16
        self.CHANNELS = 1
        self.RATE = 16000
        self.CHUNK = 1024

        # Buffer for overlapping windows
        self.buffer_size = int(self.RATE * 2)  # 2-second buffer
        self.audio_buffer = np.zeros(self.buffer_size, dtype=np.float32)
        self.buffer_index = 0

        # Threading
        self.recognition_thread = threading.Thread(target=self.recognition_worker)
        self.recognition_thread.daemon = True

        # ROS
        rospy.init_node('realtime_whisper_recognizer')
        self.command_pub = rospy.Publisher('/voice_commands', String, queue_size=10)

    def start_recognition(self):
        """Start real-time recognition with overlapping windows"""
        self.recognition_thread.start()

        # Initialize PyAudio
        audio = pyaudio.PyAudio()
        stream = audio.open(
            format=self.FORMAT,
            channels=self.CHANNELS,
            rate=self.RATE,
            input=True,
            frames_per_buffer=self.CHUNK
        )

        try:
            while not rospy.is_shutdown():
                # Read audio chunk
                data = stream.read(self.CHUNK)
                audio_chunk = np.frombuffer(data, dtype=np.int16).astype(np.float32) / 32768.0

                # Add to circular buffer
                chunk_len = len(audio_chunk)
                if self.buffer_index + chunk_len < self.buffer_size:
                    self.audio_buffer[self.buffer_index:self.buffer_index + chunk_len] = audio_chunk
                    self.buffer_index += chunk_len
                else:
                    # Wrap around the buffer
                    first_part_len = self.buffer_size - self.buffer_index
                    self.audio_buffer[self.buffer_index:] = audio_chunk[:first_part_len]
                    self.audio_buffer[:chunk_len - first_part_len] = audio_chunk[first_part_len:]
                    self.buffer_index = (self.buffer_index + chunk_len) % self.buffer_size

                # Check for speech activity (simple energy-based VAD)
                if np.mean(np.abs(audio_chunk)) > 0.01:  # Threshold for speech detection
                    # Add to recognition queue
                    start_idx = (self.buffer_index - int(self.RATE * 3)) % self.buffer_size  # 3-second window
                    audio_window = np.concatenate([
                        self.audio_buffer[start_idx:],
                        self.audio_buffer[:start_idx]
                    ])

                    self.audio_queue.put(audio_window)

                time.sleep(0.01)  # 10ms sleep for 100Hz processing

        finally:
            stream.stop_stream()
            stream.close()
            audio.terminate()

    def recognition_worker(self):
        """Background thread for recognition"""
        while not rospy.is_shutdown():
            try:
                # Get audio from queue
                audio_data = self.audio_queue.get(timeout=0.1)

                # Run Whisper transcription
                result = self.model.transcribe(audio_data, fp16=torch.cuda.is_available())

                if result["text"].strip():
                    self.command_pub.publish(result["text"].strip())

            except queue.Empty:
                continue
            except Exception as e:
                rospy.logerr(f"Recognition error: {e}")
```

## Command Ingestion and Preprocessing

### Natural Language Command Parsing

Natural language commands need to be parsed and validated before execution:

```python
import re
from typing import Dict, List, Optional
from dataclasses import dataclass

@dataclass
class ParsedCommand:
    action: str
    target: Optional[str] = None
    parameters: Dict[str, str] = None

class CommandParser:
    def __init__(self):
        # Define command patterns
        self.patterns = {
            'move': [
                r'move\s+(?P<direction>forward|backward|left|right|up|down)\s*(?P<distance>\d+(?:\.\d+)?)?\s*(?P<unit>m|cm|mm)?',
                r'go\s+(?P<direction>forward|backward|left|right|up|down)\s*(?P<distance>\d+(?:\.\d+)?)?\s*(?P<unit>m|cm|mm)?',
                r'walk\s+(?P<direction>forward|backward|left|right|up|down)\s*(?P<distance>\d+(?:\.\d+)?)?\s*(?P<unit>m|cm|mm)?',
                r'take\s+(?P<distance>\d+(?:\.\d+)?)?\s*(?P<unit>m|cm|mm)?\s+steps?\s+(?P<direction>forward|backward|left|right)'
            ],
            'navigate': [
                r'go\s+to\s+(?P<location>[a-zA-Z0-9\s]+)',
                r'navigate\s+to\s+(?P<location>[a-zA-Z0-9\s]+)',
                r'move\s+to\s+(?P<location>[a-zA-Z0-9\s]+)',
                r'walk\s+to\s+(?P<location>[a-zA-Z0-9\s]+)'
            ],
            'manipulate': [
                r'pick\s+up\s+(?P<object>[a-zA-Z0-9\s]+)',
                r'grasp\s+(?P<object>[a-zA-Z0-9\s]+)',
                r'hold\s+(?P<object>[a-zA-Z0-9\s]+)',
                r'put\s+(?P<object>[a-zA-Z0-9\s]+)\s+(?P<action>down|on|in|into)\s+(?P<target>[a-zA-Z0-9\s]+)',
                r'release\s+(?P<object>[a-zA-Z0-9\s]+)'
            ],
            'look': [
                r'look\s+at\s+(?P<target>[a-zA-Z0-9\s]+)',
                r'face\s+(?P<target>[a-zA-Z0-9\s]+)',
                r'turn\s+to\s+face\s+(?P<target>[a-zA-Z0-9\s]+)',
                r'rotate\s+to\s+see\s+(?P<target>[a-zA-Z0-9\s]+)'
            ],
            'speak': [
                r'say\s+(?P<message>.+)',
                r'speak\s+(?P<message>.+)',
                r'tell\s+(?P<message>.+)'
            ]
        }

    def parse_command(self, text: str) -> Optional[ParsedCommand]:
        """Parse natural language command into structured format"""
        text_lower = text.lower().strip()

        for action, patterns in self.patterns.items():
            for pattern in patterns:
                match = re.search(pattern, text_lower)
                if match:
                    groups = match.groupdict()

                    # Process units and distances
                    if 'distance' in groups and 'unit' in groups:
                        distance = float(groups.get('distance', 1.0))
                        unit = groups.get('unit', 'm')

                        # Convert to meters
                        if unit == 'cm':
                            distance = distance / 100.0
                        elif unit == 'mm':
                            distance = distance / 1000.0

                        groups['distance'] = distance

                    # Extract action and parameters
                    params = {k: v for k, v in groups.items() if v is not None}
                    target = params.pop('location', params.pop('object', params.pop('target', None)))

                    return ParsedCommand(
                        action=action,
                        target=target,
                        parameters=params
                    )

        # If no pattern matched, return None
        return None

# Example usage
parser = CommandParser()
command = parser.parse_command("Move forward 2 meters")
print(f"Parsed: {command}")
```

### Command Validation and Safety Filtering

Before executing voice commands, implement validation and safety checks:

```python
class CommandValidator:
    def __init__(self):
        # Define safe movement limits for humanoid robots
        self.safe_limits = {
            'max_translation_speed': 0.5,  # m/s
            'max_rotation_speed': 0.5,     # rad/s
            'max_distance': 5.0,           # m
            'max_lift_height': 1.5,        # m (above ground)
            'safe_zones': ['living_room', 'kitchen', 'office']  # Safe navigation areas
        }

        # Dangerous commands to filter
        self.dangerous_patterns = [
            r'destroy',
            r'break',
            r'hurt',
            r'damage',
            r'jump from',
            r'fall down',
            r'hit',
            r'crash'
        ]

    def validate_command(self, parsed_command: ParsedCommand) -> tuple[bool, str]:
        """Validate command for safety and feasibility"""

        # Check for dangerous commands
        if self.is_dangerous_command(parsed_command):
            return False, "Command contains potentially dangerous actions"

        # Validate movement commands
        if parsed_command.action == 'move':
            if not self.validate_move_command(parsed_command):
                return False, "Movement command exceeds safety limits"

        # Validate navigation commands
        if parsed_command.action == 'navigate':
            if not self.validate_navigation_command(parsed_command):
                return False, "Navigation destination is not in safe zones"

        # Validate manipulation commands
        if parsed_command.action == 'manipulate':
            if not self.validate_manipulation_command(parsed_command):
                return False, "Manipulation command may damage robot or environment"

        return True, "Command is valid and safe"

    def is_dangerous_command(self, parsed_command: ParsedCommand) -> bool:
        """Check if command contains dangerous patterns"""
        text = f"{parsed_command.action} {parsed_command.target} {' '.join(parsed_command.parameters.values()) if parsed_command.parameters else ''}"

        for pattern in self.dangerous_patterns:
            if re.search(pattern, text, re.IGNORECASE):
                return True
        return False

    def validate_move_command(self, parsed_command: ParsedCommand) -> bool:
        """Validate movement command parameters"""
        if parsed_command.parameters:
            # Check distance limits
            if 'distance' in parsed_command.parameters:
                distance = float(parsed_command.parameters['distance'])
                if distance > self.safe_limits['max_distance']:
                    return False

            # Check direction constraints (humanoid-specific)
            direction = parsed_command.parameters.get('direction', '')
            if direction in ['up', 'down'] and distance > 0.1:  # Humanoids have limited vertical movement
                return False

        return True

    def validate_navigation_command(self, parsed_command: ParsedCommand) -> bool:
        """Validate navigation destination"""
        if parsed_command.target:
            # Check if destination is in safe zones
            target_location = parsed_command.target.lower()
            for safe_zone in self.safe_limits['safe_zones']:
                if safe_zone in target_location:
                    return True
        return False

    def validate_manipulation_command(self, parsed_command: ParsedCommand) -> bool:
        """Validate manipulation command safety"""
        if parsed_command.target:
            # Check for fragile objects that shouldn't be manipulated
            fragile_objects = ['glass', 'mirror', 'vase', 'window', 'computer']
            target_lower = parsed_command.target.lower()

            for obj in fragile_objects:
                if obj in target_lower:
                    return False

        return True
```

## ROS 2 Integration

### Voice Command Publisher

Create a ROS 2 node that publishes recognized voice commands:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
import threading
import queue

class VoiceCommandPublisher(Node):
    def __init__(self):
        super().__init__('voice_command_publisher')

        # Publisher for raw voice commands
        self.raw_command_pub = self.create_publisher(String, '/voice/raw_command', 10)

        # Publisher for parsed commands
        self.parsed_command_pub = self.create_publisher(String, '/voice/parsed_command', 10)

        # Publisher for validated commands
        self.validated_command_pub = self.create_publisher(String, '/voice/validated_command', 10)

        # Command queue for processing
        self.command_queue = queue.Queue()

        # Command parser and validator
        self.parser = CommandParser()
        self.validator = CommandValidator()

        # Start command processing thread
        self.processing_thread = threading.Thread(target=self.process_commands)
        self.processing_thread.daemon = True
        self.processing_thread.start()

        self.get_logger().info("Voice Command Publisher initialized")

    def publish_raw_command(self, text: str):
        """Publish raw recognized text"""
        msg = String()
        msg.data = text
        self.raw_command_pub.publish(msg)

    def process_command(self, text: str):
        """Process voice command through parsing and validation pipeline"""
        # Publish raw command
        self.publish_raw_command(text)

        # Parse command
        parsed = self.parser.parse_command(text)
        if parsed:
            # Publish parsed command
            parsed_msg = String()
            parsed_msg.data = f"{parsed.action}:{parsed.target or 'None'}:{parsed.parameters or {}}"
            self.parsed_command_pub.publish(parsed_msg)

            # Validate command
            is_valid, reason = self.validator.validate_command(parsed)
            if is_valid:
                # Publish validated command
                validated_msg = String()
                validated_msg.data = f"{parsed.action}:{parsed.target or 'None'}:{parsed.parameters or {}}"
                self.validated_command_pub.publish(validated_msg)

                self.get_logger().info(f"Valid command processed: {text}")
            else:
                self.get_logger().warn(f"Invalid command rejected: {text} ({reason})")
        else:
            self.get_logger().info(f"No command parsed from: {text}")

    def process_commands(self):
        """Background thread to process commands from queue"""
        while rclpy.ok():
            try:
                text = self.command_queue.get(timeout=0.1)
                self.process_command(text)
            except queue.Empty:
                continue

    def add_command_to_queue(self, text: str):
        """Add command to processing queue"""
        self.command_queue.put(text)

def main(args=None):
    rclpy.init(args=args)

    voice_publisher = VoiceCommandPublisher()

    # Example: Simulate receiving voice commands
    # In real implementation, this would come from Whisper recognition
    sample_commands = [
        "Move forward 1 meter",
        "Go to kitchen",
        "Look at the table",
        "Pick up the red cup"
    ]

    for cmd in sample_commands:
        voice_publisher.add_command_to_queue(cmd)
        import time
        time.sleep(1)  # Simulate timing between commands

    try:
        rclpy.spin(voice_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        voice_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Voice Command Subscriber for Execution

Create a subscriber that receives validated commands and executes them:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from nav_msgs.msg import Path
from builtin_interfaces.msg import Duration
import json

class VoiceCommandExecutor(Node):
    def __init__(self):
        super().__init__('voice_command_executor')

        # Subscriber for validated commands
        self.command_sub = self.create_subscription(
            String,
            '/voice/validated_command',
            self.command_callback,
            10
        )

        # Publishers for robot control
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.joint_cmd_pub = self.create_publisher(JointState, '/joint_commands', 10)
        self.nav_goal_pub = self.create_publisher(Path, '/move_base_simple/goal', 10)

        # Robot state
        self.current_pose = None
        self.robot_joints = {}

        self.get_logger().info("Voice Command Executor initialized")

    def command_callback(self, msg):
        """Process validated voice command"""
        try:
            # Parse command from string format: action:target:params_dict
            parts = msg.data.split(':', 2)
            if len(parts) >= 2:
                action = parts[0]
                target = parts[1] if parts[1] != 'None' else None
                params_str = parts[2] if len(parts) > 2 else '{}'

                # Convert params string back to dict
                try:
                    params = json.loads(params_str.replace("'", '"'))
                except:
                    params = {}

                self.execute_command(action, target, params)
            else:
                self.get_logger().warn(f"Invalid command format: {msg.data}")

        except Exception as e:
            self.get_logger().error(f"Error processing command {msg.data}: {e}")

    def execute_command(self, action: str, target: str, params: dict):
        """Execute the parsed command"""
        self.get_logger().info(f"Executing command: {action} with target {target} and params {params}")

        if action == 'move':
            self.execute_move_command(params)
        elif action == 'navigate':
            self.execute_navigate_command(target)
        elif action == 'manipulate':
            self.execute_manipulate_command(target, params)
        elif action == 'look':
            self.execute_look_command(target)
        else:
            self.get_logger().warn(f"Unknown action: {action}")

    def execute_move_command(self, params: dict):
        """Execute movement command"""
        direction = params.get('direction', 'forward')
        distance = float(params.get('distance', 1.0))

        # Create velocity command based on direction
        cmd_vel = Twist()

        if direction == 'forward':
            cmd_vel.linear.x = 0.3  # m/s
        elif direction == 'backward':
            cmd_vel.linear.x = -0.3
        elif direction == 'left':
            cmd_vel.linear.y = 0.3
        elif direction == 'right':
            cmd_vel.linear.y = -0.3
        elif direction == 'up':
            cmd_vel.linear.z = 0.1
        elif direction == 'down':
            cmd_vel.linear.z = -0.1

        # Publish command for duration based on distance
        duration = distance / 0.3  # assuming 0.3 m/s speed
        self.move_for_duration(cmd_vel, duration)

    def move_for_duration(self, cmd_vel: Twist, duration: float):
        """Move robot with specified velocity for given duration"""
        start_time = self.get_clock().now()
        end_time = start_time + Duration(sec=int(duration), nanosec=int((duration % 1) * 1e9))

        while self.get_clock().now() < end_time and rclpy.ok():
            self.cmd_vel_pub.publish(cmd_vel)
            self.get_clock().sleep_for(Duration(nanosec=100000000))  # 100ms sleep

    def execute_navigate_command(self, target: str):
        """Execute navigation command"""
        if target:
            # In a real implementation, this would convert target location
            # to coordinates and send to navigation stack
            self.get_logger().info(f"Navigating to: {target}")
            # Implementation would go here

    def execute_manipulate_command(self, target: str, params: dict):
        """Execute manipulation command"""
        if target:
            self.get_logger().info(f"Manipulating: {target}")
            # Implementation for gripper control, arm movement, etc.
            # Would use joint control or manipulation actions

    def execute_look_command(self, target: str):
        """Execute look command"""
        if target:
            self.get_logger().info(f"Looking at: {target}")
            # Implementation for head/neck joint control
            # to orient camera toward target

def main(args=None):
    rclpy.init(args=args)

    executor = VoiceCommandExecutor()

    try:
        rclpy.spin(executor)
    except KeyboardInterrupt:
        pass
    finally:
        executor.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Performance and Latency Considerations

### Optimizing Voice Recognition Performance

For real-time humanoid applications, optimize for latency:

```python
# Optimized voice recognition node with performance considerations
import rclpy
from rclpy.node import Node
from audio_common_msgs.msg import AudioData
from std_msgs.msg import String
import numpy as np
import threading
import time
from collections import deque

class OptimizedVoiceRecognizer(Node):
    def __init__(self):
        super().__init__('optimized_voice_recognizer')

        # Performance parameters
        self.processing_interval = 0.1  # Process every 100ms
        self.audio_buffer_size = 4096  # Small buffer for low latency
        self.min_speech_energy = 0.01  # Minimum energy for speech detection

        # Audio buffer for overlapping windows
        self.audio_buffer = deque(maxlen=16000 * 2)  # 2 seconds of audio at 16kHz

        # Publishers
        self.command_pub = self.create_publisher(String, '/voice_commands', 10)

        # Audio subscriber
        self.audio_sub = self.create_subscription(
            AudioData,
            '/audio_input',
            self.audio_callback,
            10
        )

        # Threading for processing
        self.processing_lock = threading.Lock()
        self.last_process_time = time.time()

        # Initialize Whisper model (use smaller model for faster processing)
        import whisper
        self.model = whisper.load_model("tiny")  # Fastest model

        self.get_logger().info("Optimized Voice Recognizer initialized")

    def audio_callback(self, msg):
        """Process incoming audio data"""
        # Convert audio data to numpy array
        audio_data = np.frombuffer(msg.data, dtype=np.int16).astype(np.float32) / 32768.0

        # Add to buffer
        for sample in audio_data:
            self.audio_buffer.append(sample)

        # Check if we have enough audio and if it's above threshold
        if len(self.audio_buffer) >= 16000:  # At least 1 second of audio
            current_time = time.time()

            # Check for speech activity
            audio_array = np.array(list(self.audio_buffer)[-16000:])  # Last 1 second
            energy = np.mean(np.abs(audio_array))

            if energy > self.min_speech_energy and (current_time - self.last_process_time) > self.processing_interval:
                # Process audio for recognition
                with self.processing_lock:
                    self.process_audio_segment(audio_array)
                    self.last_process_time = current_time

    def process_audio_segment(self, audio_segment):
        """Process audio segment with Whisper"""
        try:
            # Run Whisper transcription
            result = self.model.transcribe(audio_segment, fp16=False)  # Use fp32 for consistency

            if result["text"].strip():
                # Publish recognized command
                cmd_msg = String()
                cmd_msg.data = result["text"].strip()
                self.command_pub.publish(cmd_msg)

                self.get_logger().info(f"Recognized: {cmd_msg.data}")

        except Exception as e:
            self.get_logger().error(f"Error in audio processing: {e}")

def main(args=None):
    rclpy.init(args=args)

    recognizer = OptimizedVoiceRecognizer()

    try:
        rclpy.spin(recognizer)
    except KeyboardInterrupt:
        pass
    finally:
        recognizer.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Summary

In this chapter, you've learned how to:
- Set up OpenAI Whisper for real-time voice recognition in robotics applications
- Parse natural language commands for humanoid robot execution
- Validate voice commands for safety and feasibility
- Integrate voice recognition with ROS 2 messaging systems
- Optimize for performance and low-latency responses

In the [next chapter](./02-llm-cognitive-planning), we'll explore how to implement cognitive planning using large language models to translate natural language goals into executable action plans for humanoid robots.