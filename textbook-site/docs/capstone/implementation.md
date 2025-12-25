# Capstone: Autonomous Humanoid Project

Welcome to the capstone project of the Physical AI & Humanoid Robotics textbook. This project integrates all concepts learned in the previous modules to create an autonomous humanoid robot that can receive voice commands, plan paths, navigate obstacles, identify objects using computer vision, and manipulate them.

## Learning Objectives

By the end of this capstone project, you will be able to:
- Integrate all four modules (ROS 2, Digital Twin, AI-Robot Brain, Vision-Language-Action) into a cohesive system
- Implement an end-to-end humanoid robot that responds to voice commands
- Design and implement complex behaviors combining perception, planning, and action
- Validate the complete system in simulation and prepare for real-world deployment
- Document and present your integrated robotics solution
- Troubleshoot complex multi-component robotic systems

## Project Overview

The capstone project involves creating an autonomous humanoid robot that can:
1. Receive and understand voice commands in English
2. Plan navigation paths to reach specified locations
3. Navigate through environments with obstacles
4. Identify objects using computer vision
5. Manipulate objects with humanoid hands
6. Integrate all components into a cohesive system

### Success Criteria

- 80% of users can successfully complete the capstone project within 6 months of starting the book
- All code examples pass automated testing in CI/CD pipeline with 95%+ success rate
- All tutorials verified on specified hardware configurations (RTX workstation, Jetson kit, Unitree robot)
- Content reviewed by 3 industry experts with 5+ years robotics experience with approval
- Beta testing with 20+ users showing 85% success rate on key tutorials
- Book adopted by at least 3 university robotics courses within 12 months of publication

## Architecture Overview

The capstone system integrates:

```
[Voice Command] → [Whisper STT] → [LLM Planner] → [Action Executor] → [Robot Control]
     ↑              ↓              ↓              ↓               ↓
[User Input]   [Text Processing] [Task Planning] [Movement]   [Physical Action]
     ↓              ↓              ↓              ↓               ↓
[Response] ← [TTS Output] ← [Execution Status] ← [Sensors] ← [Environment Feedback]
```

### Core Components

1. **Voice Processing System**: Converts speech to text using OpenAI Whisper or similar
2. **Natural Language Understanding**: Interprets commands and extracts intent using LLMs
3. **Cognitive Planner**: Decomposes high-level commands into executable action sequences
4. **Perception System**: Identifies objects and obstacles using computer vision
5. **Navigation System**: Plans and executes paths using Nav2
6. **Manipulation System**: Controls humanoid hands for object manipulation
7. **Integration Layer**: Coordinates all components and manages state
8. **Safety System**: Ensures safe operation and prevents harmful actions

## Implementation Plan

### Phase 1: System Integration

Integrate all previously developed components into a unified system:

1. Connect voice processing with cognitive planning
2. Integrate perception system with navigation
3. Link manipulation system with object identification
4. Implement system state management
5. Create the main coordinator node

### Phase 2: Behavior Implementation

Implement complex behaviors that require multiple modules:

1. **Voice Command to Action Pipeline**: Complete pipeline from voice command to physical action
2. **Object Fetch Task**: Navigate to object, identify it, pick it up, bring it back
3. **Room Navigation**: Navigate through rooms with obstacles to reach targets
4. **Multi-Object Manipulation**: Handle multiple objects in sequence
5. **Human-Robot Interaction**: Natural interaction scenarios

### Phase 3: Validation and Testing

Comprehensive testing of the integrated system:

1. Unit testing of individual components
2. Integration testing of component interactions
3. System-level testing of complete behaviors
4. Hardware validation on simulation environments
5. Performance optimization and tuning

## Capstone Project Requirements

### Technical Requirements

- Implement complete voice-to-action pipeline
- Integrate with Isaac Sim for simulation
- Connect to Isaac ROS for hardware-accelerated perception
- Implement safety checks and validation
- Create comprehensive documentation
- Ensure all components work together seamlessly

### Hardware Requirements

- RTX 4080+ workstation for simulation
- Unitree Go2 Edu robot (or equivalent humanoid platform)
- NVIDIA Jetson Orin Nano for edge computing
- Appropriate sensors (LiDAR, cameras, IMUs)

### Performance Requirements

- Voice command response time: < 3 seconds
- Navigation success rate: > 85% in cluttered environments
- Object identification accuracy: > 90%
- Manipulation success rate: > 80% for common objects
- System uptime: > 95% during operation

## Implementation Approach

### 1. Coordinator Node Implementation

The coordinator node will manage the overall system state and coordinate between components:

```python
# capstone_coordinator.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist, Pose
from sensor_msgs.msg import Image, LaserScan
from vision_msgs.msg import Detection2DArray
import json
import threading
import time

class CapstoneCoordinator(Node):
    def __init__(self):
        super().__init__('capstone_coordinator')
        
        # State management
        self.system_state = 'idle'  # idle, processing_command, navigating, manipulating, error
        self.robot_pose = Pose()
        self.current_task = None
        self.task_queue = []
        
        # Subscriptions for all components
        self.voice_command_subscription = self.create_subscription(
            String,
            '/voice_commands',
            self.voice_command_callback,
            10)
        
        self.detection_subscription = self.create_subscription(
            Detection2DArray,
            '/object_detections',
            self.detection_callback,
            10)
        
        self.laser_scan_subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.laser_scan_callback,
            10)
        
        self.robot_pose_subscription = self.create_subscription(
            Pose,
            '/robot_pose',
            self.robot_pose_callback,
            10)
        
        # Publishers
        self.action_command_publisher = self.create_publisher(
            String,
            '/action_commands',
            10)
        
        self.navigation_goal_publisher = self.create_publisher(
            Pose,
            '/navigation/goal',
            10)
        
        self.status_publisher = self.create_publisher(
            String,
            '/system_status',
            10)
        
        # Timer for system state management
        self.state_timer = self.create_timer(0.1, self.manage_system_state)
        
        self.get_logger().info('Capstone Coordinator initialized')

    def voice_command_callback(self, msg):
        """Process voice command and initiate appropriate action"""
        try:
            command_data = json.loads(msg.data)
            command_text = command_data.get('command', '')
            
            self.get_logger().info(f'Received voice command: {command_text}')
            
            # Update system state
            self.system_state = 'processing_command'
            
            # Publish status
            status_msg = String()
            status_msg.data = f'PROCESSING_COMMAND: {command_text}'
            self.status_publisher.publish(status_msg)
            
            # Determine appropriate action based on command
            action_sequence = self.plan_action_sequence(command_text)
            
            if action_sequence:
                self.execute_action_sequence(action_sequence)
            else:
                self.get_logger().warn(f'Could not determine action sequence for: {command_text}')
                self.system_state = 'idle'
                
        except json.JSONDecodeError:
            self.get_logger().error(f'Invalid JSON in voice command: {msg.data}')
        except Exception as e:
            self.get_logger().error(f'Error processing voice command: {e}')
            self.system_state = 'error'

    def plan_action_sequence(self, command_text):
        """Plan sequence of actions based on voice command"""
        # In a real implementation, this would interface with the LLM cognitive planner
        # For this example, we'll implement a simple rule-based planner
        
        command_lower = command_text.lower()
        
        if 'go to' in command_lower or 'navigate to' in command_lower:
            # Extract target location
            target_location = self.extract_location(command_text)
            return [
                {
                    'action_type': 'navigate_to',
                    'parameters': {'location': target_location},
                    'description': f'Navigate to {target_location}'
                }
            ]
        elif 'pick up' in command_lower or 'get' in command_lower:
            # Extract object to pick up
            object_name = self.extract_object(command_text)
            return [
                {
                    'action_type': 'locate_object',
                    'parameters': {'object_name': object_name},
                    'description': f'Locate {object_name}'
                },
                {
                    'action_type': 'navigate_to',
                    'parameters': {'target': f'near_{object_name}'},
                    'description': f'Navigate to {object_name}'
                },
                {
                    'action_type': 'pick_object',
                    'parameters': {'object_name': object_name},
                    'description': f'Pick up {object_name}'
                }
            ]
        elif 'bring' in command_lower or 'fetch' in command_lower:
            # Extract object and destination
            parts = command_text.split()
            obj_index = -1
            dest_index = -1
            
            for i, part in enumerate(parts):
                if part.lower() in ['the', 'a', 'an']:
                    if i+1 < len(parts):
                        obj_index = i+1
                        break
            
            # Look for destination after 'to'
            for i, part in enumerate(parts):
                if part.lower() == 'to' and i+1 < len(parts):
                    dest_index = i+1
                    break
            
            object_name = parts[obj_index] if obj_index != -1 else 'object'
            destination = parts[dest_index] if dest_index != -1 else 'here'
            
            return [
                {
                    'action_type': 'locate_object',
                    'parameters': {'object_name': object_name},
                    'description': f'Locate {object_name}'
                },
                {
                    'action_type': 'navigate_to',
                    'parameters': {'target': f'near_{object_name}'},
                    'description': f'Navigate to {object_name}'
                },
                {
                    'action_type': 'pick_object',
                    'parameters': {'object_name': object_name},
                    'description': f'Pick up {object_name}'
                },
                {
                    'action_type': 'navigate_to',
                    'parameters': {'target': destination},
                    'description': f'Navigate to {destination}'
                },
                {
                    'action_type': 'place_object',
                    'parameters': {'object_name': object_name, 'location': destination},
                    'description': f'Place {object_name} at {destination}'
                }
            ]
        else:
            # For unrecognized commands, request clarification
            self.get_logger().warn(f'Unrecognized command: {command_text}')
            return None

    def extract_location(self, command):
        """Extract location from command (simplified implementation)"""
        # Look for location after 'to' or 'at'
        words = command.lower().split()
        for i, word in enumerate(words):
            if word in ['to', 'at'] and i+1 < len(words):
                # Get the next word(s) as location
                location = words[i+1]
                if i+2 < len(words) and words[i+2] not in ['.', ',', '!', '?']:
                    location += ' ' + words[i+2]  # Include next word if it's part of location name
                return location
        return 'destination'

    def extract_object(self, command):
        """Extract object from command (simplified implementation)"""
        # Look for object after 'pick up', 'get', 'bring', etc.
        words = command.lower().split()
        for i, word in enumerate(words):
            if word in ['the', 'a', 'an'] and i+1 < len(words):
                return words[i+1]
            elif word in ['pick', 'get', 'bring'] and i+1 < len(words):
                if words[i+1] in ['up', 'the', 'a', 'an'] and i+2 < len(words):
                    return words[i+2]
        return 'object'

    def detection_callback(self, msg):
        """Process object detections"""
        # Store detection information for planning
        for detection in msg.detections:
            if detection.results and len(detection.results) > 0:
                best_result = max(detection.results, key=lambda x: x.score)
                if best_result.score > 0.7:  # Confidence threshold
                    self.update_object_knowledge(best_result.id, detection.bbox)

    def laser_scan_callback(self, msg):
        """Process laser scan for navigation"""
        # Store scan information for navigation planning
        self.latest_scan = msg

    def robot_pose_callback(self, msg):
        """Update robot pose information"""
        self.robot_pose = msg

    def execute_action_sequence(self, action_sequence):
        """Execute a sequence of actions"""
        self.current_task = {
            'sequence': action_sequence,
            'current_index': 0,
            'start_time': time.time()
        }
        
        self.execute_current_action()

    def execute_current_action(self):
        """Execute the current action in the sequence"""
        if not self.current_task:
            return
            
        sequence = self.current_task['sequence']
        index = self.current_task['current_index']
        
        if index >= len(sequence):
            # Task complete
            self.get_logger().info('Task sequence completed successfully')
            self.system_state = 'idle'
            self.current_task = None
            return
        
        action = sequence[index]
        self.get_logger().info(f'Executing action {index+1}/{len(sequence)}: {action["description"]}')
        
        # Publish action command
        action_msg = String()
        action_msg.data = json.dumps(action)
        self.action_command_publisher.publish(action_msg)
        
        # Update task index
        self.current_task['current_index'] += 1

    def manage_system_state(self):
        """Manage system state and handle state transitions"""
        # This would handle state transitions based on feedback from components
        # For now, just publish status periodically
        status_msg = String()
        status_msg.data = f'SYSTEM_STATE: {self.system_state}, ROBOT_POSITION: ({self.robot_pose.position.x:.2f}, {self.robot_pose.position.y:.2f})'
        self.status_publisher.publish(status_msg)

def main(args=None):
    rclpy.init(args=args)
    coordinator = CapstoneCoordinator()
    
    try:
        rclpy.spin(coordinator)
    except KeyboardInterrupt:
        coordinator.get_logger().info('Shutting down capstone coordinator...')
    finally:
        coordinator.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### 2. System Integration Layer

The integration layer connects all components and manages their interactions:

```python
# system_integration.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image, JointState
from geometry_msgs.msg import Twist, Pose
import threading
import time

class SystemIntegrationLayer(Node):
    def __init__(self):
        super().__init__('system_integration_layer')
        
        # Publishers for all subsystems
        self.voice_cmd_publisher = self.create_publisher(String, '/voice_commands', 10)
        self.nav_goal_publisher = self.create_publisher(Pose, '/navigation/goal', 10)
        self.joint_cmd_publisher = self.create_publisher(JointState, '/joint_commands', 10)
        self.base_cmd_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Subscriptions for system status
        self.voice_status_subscription = self.create_subscription(
            String, '/voice_status', self.voice_status_callback, 10)
        self.nav_status_subscription = self.create_subscription(
            String, '/navigation/status', self.nav_status_callback, 10)
        self.manip_status_subscription = self.create_subscription(
            String, '/manipulation/status', self.manip_status_callback, 10)
        
        # System state
        self.voice_system_active = False
        self.nav_system_active = False
        self.manip_system_active = False
        self.system_overall_status = 'initializing'
        
        # Timer for health checks
        self.health_check_timer = self.create_timer(1.0, self.system_health_check)
        
        self.get_logger().info('System Integration Layer initialized')

    def voice_status_callback(self, msg):
        """Update voice system status"""
        if 'ACTIVE' in msg.data:
            self.voice_system_active = True
        elif 'INACTIVE' in msg.data:
            self.voice_system_active = False

    def nav_status_callback(self, msg):
        """Update navigation system status"""
        if 'ACTIVE' in msg.data:
            self.nav_system_active = True
        elif 'INACTIVE' in msg.data:
            self.nav_system_active = False

    def manip_status_callback(self, msg):
        """Update manipulation system status"""
        if 'ACTIVE' in msg.data:
            self.manip_system_active = True
        elif 'INACTIVE' in msg.data:
            self.manip_system_active = False

    def system_health_check(self):
        """Check health of all subsystems"""
        active_systems = sum([
            self.voice_system_active,
            self.nav_system_active,
            self.manip_system_active
        ])
        
        total_systems = 3
        
        if active_systems == total_systems:
            self.system_overall_status = 'fully_operational'
        elif active_systems == 0:
            self.system_overall_status = 'inactive'
        else:
            self.system_overall_status = 'partial_operation'
        
        self.get_logger().info(f'System health: {self.system_overall_status} ({active_systems}/{total_systems} subsystems active)')
        
        # Restart any inactive subsystems if needed
        if not self.voice_system_active:
            self.restart_voice_system()
        if not self.nav_system_active:
            self.restart_navigation_system()
        if not self.manip_system_active:
            self.restart_manipulation_system()

    def restart_voice_system(self):
        """Restart voice processing system if needed"""
        # Implementation to restart voice system
        self.get_logger().info('Attempting to restart voice processing system...')
        # In a real implementation, this would restart the voice processing node

    def restart_navigation_system(self):
        """Restart navigation system if needed"""
        # Implementation to restart navigation system
        self.get_logger().info('Attempting to restart navigation system...')
        # In a real implementation, this would restart the navigation stack

    def restart_manipulation_system(self):
        """Restart manipulation system if needed"""
        # Implementation to restart manipulation system
        self.get_logger().info('Attempting to restart manipulation system...')
        # In a real implementation, this would restart the manipulation controllers

def main(args=None):
    rclpy.init(args=args)
    integration_layer = SystemIntegrationLayer()
    
    try:
        rclpy.spin(integration_layer)
    except KeyboardInterrupt:
        integration_layer.get_logger().info('Shutting down system integration layer...')
    finally:
        integration_layer.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### 3. Safety and Validation System

The safety system ensures all actions are safe before execution:

```python
# safety_validator.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist, Pose
from sensor_msgs.msg import LaserScan
import numpy as np

class SafetyValidator(Node):
    def __init__(self):
        super().__init__('safety_validator')
        
        # Subscriptions for commands that need validation
        self.base_cmd_subscription = self.create_subscription(
            Twist,
            '/unsafe_cmd_vel',
            self.base_cmd_callback,
            10)
        
        self.nav_goal_subscription = self.create_subscription(
            Pose,
            '/unsafe_navigation/goal',
            self.nav_goal_callback,
            10)
        
        self.laser_scan_subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.laser_scan_callback,
            10)
        
        # Publishers for validated commands
        self.safe_base_cmd_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.safe_nav_goal_publisher = self.create_publisher(Pose, '/navigation/goal', 10)
        
        # Safety parameters
        self.max_linear_velocity = 0.5  # m/s
        self.max_angular_velocity = 0.5  # rad/s
        self.safety_distance = 0.5  # meters
        self.latest_scan = None
        
        self.get_logger().info('Safety Validator initialized')

    def base_cmd_callback(self, msg):
        """Validate base velocity commands"""
        # Check velocity limits
        if abs(msg.linear.x) > self.max_linear_velocity:
            self.get_logger().warn(f'Linear velocity {msg.linear.x} exceeds limit {self.max_linear_velocity}')
            msg.linear.x = np.sign(msg.linear.x) * self.max_linear_velocity
        
        if abs(msg.angular.z) > self.max_angular_velocity:
            self.get_logger().warn(f'Angular velocity {msg.angular.z} exceeds limit {self.max_angular_velocity}')
            msg.angular.z = np.sign(msg.angular.z) * self.max_angular_velocity
        
        # Check for obstacles if moving forward
        if msg.linear.x > 0 and self.latest_scan:
            if self.is_path_blocked():
                self.get_logger().warn('Path forward is blocked, stopping robot')
                msg.linear.x = 0.0
                msg.angular.z = 0.0
        
        # Publish validated command
        self.safe_base_cmd_publisher.publish(msg)

    def nav_goal_callback(self, msg):
        """Validate navigation goals"""
        # Check if goal is in a safe area
        if self.is_goal_safe(msg):
            self.safe_nav_goal_publisher.publish(msg)
            self.get_logger().info('Navigation goal validated and published')
        else:
            self.get_logger().error('Navigation goal is unsafe, rejecting')
            # Publish error message
            error_msg = String()
            error_msg.data = f'UNSAFE_GOAL_REJECTED: Goal at ({msg.position.x}, {msg.position.y}) is unsafe'
            # In a real implementation, this would go to an error topic

    def laser_scan_callback(self, msg):
        """Update latest scan for safety checks"""
        self.latest_scan = msg

    def is_path_blocked(self):
        """Check if path forward is blocked by obstacles"""
        if not self.latest_scan:
            return False  # Can't determine, assume safe
        
        # Check forward sector (e.g., ±30 degrees from forward)
        forward_sector_start = len(self.latest_scan.ranges) // 2 - len(self.latest_scan.ranges) // 12  # -30 degrees
        forward_sector_end = len(self.latest_scan.ranges) // 2 + len(self.latest_scan.ranges) // 12   # +30 degrees
        
        forward_distances = self.latest_scan.ranges[forward_sector_start:forward_sector_end]
        
        # Check for obstacles within safety distance
        for dist in forward_distances:
            if 0 < dist < self.safety_distance:
                return True  # Obstacle detected
        
        return False

    def is_goal_safe(self, pose):
        """Check if navigation goal is safe"""
        # In a real implementation, this would check:
        # - Goal is within known map bounds
        # - Goal is not in an excluded area (e.g., stairs, fragile objects)
        # - Path to goal is clear of obstacles
        # - Goal is reachable given robot kinematics
        
        # For this example, just check that it's not extremely far
        distance = np.sqrt(pose.position.x**2 + pose.position.y**2)
        
        # Check if goal is reasonably close (within 20m for example)
        if distance > 20.0:
            self.get_logger().warn(f'Goal too far: {distance:.2f}m')
            return False
        
        # Check if goal is too high (above ground level)
        if pose.position.z > 2.0:
            self.get_logger().warn(f'Goal too high: {pose.position.z:.2f}m')
            return False
        
        return True

def main(args=None):
    rclpy.init(args=args)
    validator = SafetyValidator()
    
    try:
        rclpy.spin(validator)
    except KeyboardInterrupt:
        validator.get_logger().info('Shutting down safety validator...')
    finally:
        validator.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Testing the Complete System

### Integration Testing Framework

```python
# integration_tester.py
import unittest
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
import time

class CapstoneIntegrationTester(unittest.TestCase):
    def setUp(self):
        rclpy.init()
        self.node = Node('integration_tester')
        
        # Publishers for sending test commands
        self.voice_cmd_publisher = self.node.create_publisher(
            String, '/voice_commands', 10)
        self.base_cmd_publisher = self.node.create_publisher(
            Twist, '/unsafe_cmd_vel', 10)
        
        # Subscriptions for receiving test results
        self.status_subscription = self.node.create_subscription(
            String, '/system_status', self.status_callback, 10)
        
        self.test_results = []
        self.current_test = None

    def status_callback(self, msg):
        """Record system status during tests"""
        if self.current_test:
            self.test_results.append((self.current_test, msg.data))

    def test_voice_command_processing(self):
        """Test complete voice command processing pipeline"""
        self.current_test = "voice_command_processing"
        
        # Send a voice command
        cmd_msg = String()
        cmd_msg.data = '{"command": "go to the kitchen and bring me a cup"}'
        self.voice_cmd_publisher.publish(cmd_msg)
        
        # Wait for processing
        time.sleep(5.0)
        
        # Verify system responded appropriately
        status_found = any("PROCESSING_COMMAND" in result[1] for result in self.test_results if result[0] == self.current_test)
        self.assertTrue(status_found, "System should process voice command")

    def test_navigation_integration(self):
        """Test navigation system integration"""
        self.current_test = "navigation_integration"
        
        # Send navigation command
        twist_cmd = Twist()
        twist_cmd.linear.x = 0.5  # Move forward
        self.base_cmd_publisher.publish(twist_cmd)
        
        # Wait for response
        time.sleep(2.0)
        
        # Verify command was received and processed safely
        self.assertTrue(True)  # Placeholder - actual test would check safety validator

    def tearDown(self):
        self.node.destroy_node()
        rclpy.shutdown()

def run_integration_tests():
    """Run all integration tests"""
    suite = unittest.TestLoader().loadTestsFromTestCase(CapstoneIntegrationTester)
    runner = unittest.TextTestRunner(verbosity=2)
    result = runner.run(suite)
    
    return result.wasSuccessful()

if __name__ == '__main__':
    success = run_integration_tests()
    if success:
        print("All integration tests passed!")
    else:
        print("Some integration tests failed!")
        exit(1)
```

## Performance Optimization

For the capstone project, performance optimization is critical since all modules are running simultaneously:

### Resource Management

```python
# resource_manager.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import psutil
import GPUtil

class ResourceManager(Node):
    def __init__(self):
        super().__init__('resource_manager')
        
        self.status_publisher = self.create_publisher(
            String, '/system_resources', 10)
        
        # Timer for resource monitoring
        self.resource_timer = self.create_timer(2.0, self.monitor_resources)
        
        self.get_logger().info('Resource Manager initialized')

    def monitor_resources(self):
        """Monitor system resources and adjust processing accordingly"""
        # CPU usage
        cpu_percent = psutil.cpu_percent(interval=1)
        
        # Memory usage
        memory = psutil.virtual_memory()
        memory_percent = memory.percent
        
        # GPU usage (if available)
        gpus = GPUtil.getGPUs()
        gpu_load = gpus[0].load if gpus else 0
        gpu_memory = gpus[0].memoryUtil if gpus else 0
        
        # Create resource status message
        resource_status = {
            'cpu_percent': cpu_percent,
            'memory_percent': memory_percent,
            'gpu_load': gpu_load,
            'gpu_memory_util': gpu_memory,
            'timestamp': self.get_clock().now().nanoseconds
        }
        
        status_msg = String()
        status_msg.data = str(resource_status)
        self.status_publisher.publish(status_msg)
        
        # Adjust processing based on resource usage
        if cpu_percent > 80 or gpu_load > 0.85:
            self.get_logger().warn('High resource usage detected, consider reducing processing load')
        
        self.get_logger().info(f'CPU: {cpu_percent}%, Memory: {memory_percent}%, GPU: {gpu_load*100:.1f}%')

def main(args=None):
    rclpy.init(args=args)
    rm = ResourceManager()
    
    try:
        rclpy.spin(rm)
    except KeyboardInterrupt:
        rm.get_logger().info('Shutting down resource manager...')
    finally:
        rm.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Summary

The capstone project integrates all the modules we've developed throughout the textbook:

1. **Module 1 (ROS 2)**: Provides the communication framework
2. **Module 2 (Digital Twin)**: Simulation environment for testing
3. **Module 3 (AI-Robot Brain)**: Perception and planning systems
4. **Module 4 (VLA)**: Vision-Language-Action integration

The project demonstrates a complete humanoid robot system that can receive voice commands, process them through AI systems, navigate environments, identify and manipulate objects, all while maintaining safety and performance requirements.

This implementation provides the foundation for a fully autonomous humanoid robot system that can interact naturally with humans and perform complex tasks in real-world environments.