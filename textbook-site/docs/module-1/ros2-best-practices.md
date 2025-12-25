# ROS 2 Best Practices for Humanoid Robotics

This section outlines best practices for developing ROS 2 applications specifically for humanoid robotics. These practices are designed to ensure robust, maintainable, and efficient code that can handle the complexities of embodied AI systems.

## Learning Objectives

By the end of this section, you will be able to:
- Apply ROS 2 architectural patterns for humanoid robotics applications
- Design effective communication patterns between robot components
- Implement proper error handling and fault tolerance in robotic systems
- Structure packages and nodes for maintainability and reusability
- Optimize performance for real-time robotic applications
- Follow security best practices for robotic systems
- Implement proper testing and validation procedures

## Architectural Patterns for Humanoid Robotics

### 1. Component-Based Architecture

For humanoid robots, it's essential to decompose the system into logical components that can be developed, tested, and maintained independently:

```python
# File: humanoid_components/humanoid_components/body_controller.py

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist
from humanoid_msgs.msg import BodyState, GaitCommand


class BodyControllerNode(Node):

    def __init__(self):
        super().__init__('body_controller')
        
        # QoS profiles for different types of data
        sensor_qos = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE)
        
        command_qos = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE)
        
        # Publishers for different body parts
        self.left_arm_cmd_pub = self.create_publisher(JointState, 'left_arm/commands', command_qos)
        self.right_arm_cmd_pub = self.create_publisher(JointState, 'right_arm/commands', command_qos)
        self.left_leg_cmd_pub = self.create_publisher(JointState, 'left_leg/commands', command_qos)
        self.right_leg_cmd_pub = self.create_publisher(JointState, 'right_leg/commands', command_qos)
        
        # Subscribers for sensor data
        self.joint_state_sub = self.create_subscription(
            JointState, 'joint_states', self.joint_state_callback, sensor_qos)
        self.body_cmd_sub = self.create_subscription(
            Twist, 'body_cmd', self.body_command_callback, command_qos)
        self.gait_cmd_sub = self.create_subscription(
            GaitCommand, 'gait_cmd', self.gait_command_callback, command_qos)
        
        # Internal state
        self.current_joint_states = JointState()
        self.body_command = Twist()
        
        self.get_logger().info('Body Controller initialized')

    def joint_state_callback(self, msg):
        """Handle joint state updates"""
        self.current_joint_states = msg
        # Update internal state and possibly trigger actions

    def body_command_callback(self, msg):
        """Handle body movement commands"""
        self.body_command = msg
        # Process command and generate appropriate joint commands

    def gait_command_callback(self, msg):
        """Handle gait commands"""
        # Process gait command and generate appropriate joint trajectories
        pass

    def destroy_node(self):
        """Clean up resources"""
        self.get_logger().info('Shutting down body controller...')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    
    controller = BodyControllerNode()
    
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        controller.get_logger().info('Shutting down...')
    finally:
        controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### 2. Layered Architecture

Organize your humanoid robot system in layers with clear responsibilities:

```
┌─────────────────────────────────────────┐
│              Presentation Layer          │  ← User interfaces, dashboards
├─────────────────────────────────────────┤
│              Application Layer           │  ← High-level behaviors, tasks
├─────────────────────────────────────────┤
│               Control Layer              │  ← Low-level control, motion planning
├─────────────────────────────────────────┤
│              Hardware Abstraction        │  ← Device drivers, interfaces
└─────────────────────────────────────────┘
```

### 3. Service-Oriented Architecture

For humanoid robots, use services for high-level commands that have clear start and end states:

```python
# File: humanoid_behaviors/humanoid_behaviors/walk_service.py

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from humanoid_msgs.srv import WalkToPose
from humanoid_msgs.action import WalkToPose as WalkToPoseAction
from geometry_msgs.msg import Pose
from std_msgs.msg import Bool


class WalkService(Node):

    def __init__(self):
        super().__init__('walk_service')
        
        # Service for immediate walk commands
        self.walk_service = self.create_service(
            WalkToPose, 
            'walk_to_pose', 
            self.walk_to_pose_callback)
        
        # Action for long-running walk tasks with feedback
        self.walk_action_server = ActionServer(
            self,
            WalkToPoseAction,
            'walk_to_pose_action',
            self.execute_walk_action)
        
        # Publisher for walking status
        self.walking_status_pub = self.create_publisher(Bool, 'walking_status', 10)
        
        self.is_walking = False

    def walk_to_pose_callback(self, request, response):
        """Handle immediate walk command"""
        target_pose = request.target_pose
        self.get_logger().info(f'Walking to pose: {target_pose}')
        
        try:
            # Execute walk command synchronously
            success = self.execute_walk_command(target_pose)
            response.success = success
            response.message = "Walk command executed" if success else "Walk command failed"
        except Exception as e:
            response.success = False
            response.message = f"Error: {str(e)}"
        
        return response

    def execute_walk_action(self, goal_handle):
        """Handle walk action with feedback"""
        target_pose = goal_handle.request.target_pose
        self.get_logger().info(f'Executing walk action to pose: {target_pose}')
        
        feedback_msg = WalkToPoseAction.Feedback()
        result = WalkToPoseAction.Result()
        
        # Simulate walking progress
        for i in range(0, 101, 10):
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                result.success = False
                result.message = "Walk canceled"
                return result
            
            # Update feedback
            feedback_msg.progress = i
            goal_handle.publish_feedback(feedback_msg)
            
            # Simulate walking
            self.execute_walk_step(i)
        
        goal_handle.succeed()
        result.success = True
        result.message = "Successfully walked to pose"
        return result

    def execute_walk_command(self, target_pose):
        """Execute the actual walking command"""
        # Implementation of walking logic
        # This would typically involve:
        # 1. Path planning
        # 2. Gait generation
        # 3. Balance control
        # 4. Motor commands
        return True  # Simulated success

    def execute_walk_step(self, progress):
        """Execute one step of the walk"""
        # Implementation of one step in the walk
        pass


def main(args=None):
    rclpy.init(args=args)
    
    walk_service = WalkService()
    
    try:
        rclpy.spin(walk_service)
    except KeyboardInterrupt:
        walk_service.get_logger().info('Shutting down walk service...')
    finally:
        walk_service.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## Communication Patterns Best Practices

### 1. Proper QoS Configuration

Different types of data require different Quality of Service settings:

```python
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy, QoSDurabilityPolicy

# For sensor data (high frequency, can tolerate some loss)
SENSOR_QOS = QoSProfile(
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=5,
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    durability=QoSDurabilityPolicy.VOLATILE)

# For commands (low frequency, must be reliable)
COMMAND_QOS = QoSProfile(
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=10,
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.VOLATILE)

# For configuration data (need to keep for late-joining nodes)
CONFIG_QOS = QoSProfile(
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=1,
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
```

### 2. Efficient Message Handling

For humanoid robots that process large amounts of sensor data:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2, JointState
from builtin_interfaces.msg import Time
import time
from threading import Lock


class EfficientSensorNode(Node):

    def __init__(self):
        super().__init__('efficient_sensor_node')
        
        # Use threading lock to protect shared data
        self.data_lock = Lock()
        
        # Use small queue sizes for performance
        self.image_sub = self.create_subscription(
            Image, 'camera/image_raw', self.image_callback, 1)
        
        self.pointcloud_sub = self.create_subscription(
            PointCloud2, 'lidar/points', self.pointcloud_callback, 1)
        
        self.joint_state_sub = self.create_subscription(
            JointState, 'joint_states', self.joint_state_callback, 5)
        
        # Use timers for processing to decouple from sensor frequencies
        self.processing_timer = self.create_timer(0.05, self.process_sensor_data)  # 20 Hz processing
        
        # Internal buffers for sensor data
        self.latest_image = None
        self.latest_pointcloud = None
        self.joint_states = None
        
        # Performance tracking
        self.processing_times = []

    def image_callback(self, msg):
        """Handle incoming images efficiently"""
        # Store reference to image, don't copy large data
        with self.data_lock:
            self.latest_image = msg

    def pointcloud_callback(self, msg):
        """Handle incoming point clouds efficiently"""
        with self.data_lock:
            self.latest_pointcloud = msg

    def joint_state_callback(self, msg):
        """Handle joint states"""
        with self.data_lock:
            self.joint_states = msg

    def process_sensor_data(self):
        """Process sensor data at a controlled rate"""
        start_time = time.time()
        
        with self.data_lock:
            # Work with local copies to minimize lock time
            image = self.latest_image
            pointcloud = self.latest_pointcloud
            joints = self.joint_states
        
        if image is not None:
            # Process image data
            self.process_image(image)
        
        if pointcloud is not None:
            # Process point cloud data
            self.process_pointcloud(pointcloud)
        
        if joints is not None:
            # Process joint data
            self.update_control(joints)
        
        # Track performance
        processing_time = time.time() - start_time
        self.processing_times.append(processing_time)
        
        if len(self.processing_times) > 100:
            self.processing_times.pop(0)
        
        avg_processing_time = sum(self.processing_times) / len(self.processing_times)
        self.get_logger().debug(f'Average processing time: {avg_processing_time:.4f}s')
```

## Error Handling and Fault Tolerance

### 1. Graceful Degradation

Humanoid robots need to handle component failures gracefully:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from enum import Enum


class RobotState(Enum):
    OPERATIONAL = "operational"
    DEGRADED = "degraded"
    SAFETY_MODE = "safety_mode"
    EMERGENCY_STOP = "emergency_stop"


class FaultTolerantNode(Node):

    def __init__(self):
        super().__init__('fault_tolerant_node')
        
        # Initialize robot state
        self.robot_state = RobotState.OPERATIONAL
        self.failed_sensors = set()
        self.failed_actuators = set()
        
        # Publishers for status
        self.state_publisher = self.create_publisher(String, 'robot_state', 10)
        self.diagnostic_publisher = self.create_publisher(String, 'diagnostics', 10)
        
        # Subscriptions
        self.joint_state_sub = self.create_subscription(
            JointState, 'joint_states', self.joint_state_callback, 10)
        
        # Timer for state monitoring
        self.monitor_timer = self.create_timer(1.0, self.monitor_system_state)
        
        self.get_logger().info('Fault tolerant node initialized')

    def joint_state_callback(self, msg):
        """Handle joint states with error detection"""
        try:
            # Validate joint state message
            if not self.validate_joint_state(msg):
                self.handle_invalid_data(msg)
                return
            
            # Process valid joint state
            self.process_joint_state(msg)
            
        except Exception as e:
            self.get_logger().error(f'Error processing joint state: {e}')
            self.handle_sensor_error('joint_states', str(e))

    def validate_joint_state(self, msg):
        """Validate joint state message"""
        # Check for valid timestamps
        if msg.header.stamp.sec == 0 and msg.header.stamp.nanosec == 0:
            return False
        
        # Check for NaN or infinite values
        for pos in msg.position:
            if not (isinstance(pos, float) and not (pos != pos or pos == float('inf'))):
                return False
        
        return True

    def handle_invalid_data(self, msg):
        """Handle invalid sensor data"""
        self.get_logger().warn('Invalid sensor data received, using previous values')
        # Implement fallback behavior

    def handle_sensor_error(self, sensor_name, error_msg):
        """Handle sensor errors"""
        self.failed_sensors.add(sensor_name)
        self.update_robot_state()
        
        self.get_logger().error(f'Sensor {sensor_name} failed: {error_msg}')

    def handle_actuator_error(self, actuator_name, error_msg):
        """Handle actuator errors"""
        self.failed_actuators.add(actuator_name)
        self.update_robot_state()
        
        self.get_logger().error(f'Actuator {actuator_name} failed: {error_msg}')

    def update_robot_state(self):
        """Update robot state based on failures"""
        if len(self.failed_actuators) > 5:  # Critical number of failed actuators
            self.robot_state = RobotState.EMERGENCY_STOP
        elif len(self.failed_sensors) > 3 or len(self.failed_actuators) > 2:
            self.robot_state = RobotState.SAFETY_MODE
        elif len(self.failed_sensors) > 0 or len(self.failed_actuators) > 0:
            self.robot_state = RobotState.DEGRADED
        else:
            self.robot_state = RobotState.OPERATIONAL
        
        # Publish new state
        state_msg = String()
        state_msg.data = self.robot_state.value
        self.state_publisher.publish(state_msg)

    def monitor_system_state(self):
        """Monitor system state and publish diagnostics"""
        diag_msg = String()
        diag_msg.data = (
            f"State: {self.robot_state.value}, "
            f"Failed sensors: {len(self.failed_sensors)}, "
            f"Failed actuators: {len(self.failed_actuators)}"
        )
        self.diagnostic_publisher.publish(diag_msg)

    def get_reduced_functionality(self):
        """Return reduced functionality based on current state"""
        if self.robot_state == RobotState.EMERGENCY_STOP:
            return {"locomotion": False, "manipulation": False, "navigation": False}
        elif self.robot_state == RobotState.SAFETY_MODE:
            return {"locomotion": False, "manipulation": True, "navigation": False}
        elif self.robot_state == RobotState.DEGRADED:
            return {"locomotion": True, "manipulation": True, "navigation": True}
        else:  # OPERATIONAL
            return {"locomotion": True, "manipulation": True, "navigation": True}
```

## Performance Optimization

### 1. Memory Management

For resource-constrained humanoid robots:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from collections import deque
import gc
import psutil
import os


class MemoryEfficientNode(Node):

    def __init__(self):
        super().__init__('memory_efficient_node')
        
        # Use deques with maximum lengths to limit memory usage
        self.image_buffer = deque(maxlen=5)  # Only keep last 5 images
        
        self.image_sub = self.create_subscription(
            Image, 'camera/image_raw', self.image_callback, 1)
        
        # Memory monitoring
        self.memory_timer = self.create_timer(5.0, self.check_memory_usage)
        
        # Process monitoring
        self.process = psutil.Process(os.getpid())

    def image_callback(self, msg):
        """Process image efficiently to minimize memory usage"""
        # Don't store the entire image message if we don't need it
        # Only keep what's necessary for processing
        
        # Add to buffer
        self.image_buffer.append(msg)
        
        # Process the image (do the actual work here)
        self.process_image(msg)
        
        # Trigger garbage collection if needed
        self.maybe_gc()

    def process_image(self, msg):
        """Process image data"""
        # Implementation of image processing
        # Should avoid creating unnecessary copies of large data
        pass

    def maybe_gc(self):
        """Trigger garbage collection when memory usage is high"""
        memory_percent = self.process.memory_percent()
        if memory_percent > 75:  # If using more than 75% of allowed memory
            self.get_logger().warn(f'High memory usage: {memory_percent:.1f}%, running GC')
            gc.collect()

    def check_memory_usage(self):
        """Monitor memory usage"""
        memory_info = self.process.memory_info()
        memory_mb = memory_info.rss / 1024 / 1024
        
        self.get_logger().debug(f'Memory usage: {memory_mb:.1f} MB')
        
        if memory_mb > 1000:  # More than 1GB
            self.get_logger().warn(f'High memory usage: {memory_mb:.1f} MB')
```

### 2. Real-time Considerations

For humanoid robots that need to maintain balance and respond quickly:

```python
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import Float64MultiArray
from builtin_interfaces.msg import Duration
import time


class RealTimeNode(Node):

    def __init__(self):
        super().__init__('real_time_node')
        
        # Use reliable QoS for critical control messages
        control_qos = QoSProfile(depth=1, reliability=2)  # RELIABLE
        
        self.control_pub = self.create_publisher(
            Float64MultiArray, 'joint_group_position_controller/commands', control_qos)
        
        # Timer with fixed rate for real-time control
        # Using smaller time intervals for more responsive control
        self.control_timer = self.create_timer(0.01, self.control_loop)  # 100 Hz control loop
        
        # Track timing for real-time performance
        self.loop_times = deque(maxlen=100)
        self.target_loop_time = 0.01  # 10 ms target
        self.tolerance = 0.001  # 1 ms tolerance

    def control_loop(self):
        """Main control loop with timing constraints"""
        start_time = time.time()
        
        # Do critical control calculations
        joint_commands = self.compute_control_commands()
        
        # Publish commands
        cmd_msg = Float64MultiArray()
        cmd_msg.data = joint_commands
        self.control_pub.publish(cmd_msg)
        
        # Calculate loop time
        loop_time = time.time() - start_time
        self.loop_times.append(loop_time)
        
        # Check if we're meeting real-time constraints
        if loop_time > (self.target_loop_time + self.tolerance):
            avg_loop_time = sum(self.loop_times) / len(self.loop_times)
            self.get_logger().warn(
                f'Control loop exceeded timing: {loop_time*1000:.1f}ms '
                f'(avg: {avg_loop_time*1000:.1f}ms, target: {self.target_loop_time*1000:.1f}ms)'
            )

    def compute_control_commands(self):
        """Compute joint commands - this must be fast and deterministic"""
        # Implementation of control algorithm
        # Should avoid dynamic memory allocation and unpredictable operations
        commands = [0.0] * 12  # Example: 12 DOF humanoid
        return commands
```

## Security Best Practices

### 1. Secure Communication

```python
# Example of securing ROS 2 communications
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class SecureNode(Node):

    def __init__(self):
        super().__init__('secure_node')
        
        # For production systems, implement secure communication
        # This is a simplified example - real systems would need proper authentication
        
        # Use encrypted communication channels when possible
        # Validate message authenticity
        # Implement access controls
        
        self.publisher = self.create_publisher(String, 'secure_topic', 10)
        self.subscription = self.create_subscription(
            String, 'secure_input', self.secure_callback, 10)
        
        self.get_logger().info('Secure node initialized')

    def secure_callback(self, msg):
        """Handle messages with security checks"""
        # Validate message content
        if not self.is_valid_message(msg):
            self.get_logger().warn('Invalid message received, discarding')
            return
        
        # Process message
        self.process_secure_message(msg)

    def is_valid_message(self, msg):
        """Validate message security"""
        # Implement validation logic
        # Check message signatures, source authentication, etc.
        return True  # Simplified for example

    def process_secure_message(self, msg):
        """Process validated message"""
        # Implementation of secure message processing
        pass
```

## Testing and Validation

### 1. Component Testing

```python
# File: test/test_body_controller.py
import unittest
import rclpy
from rclpy.executors import SingleThreadedExecutor
from humanoid_components.body_controller import BodyControllerNode
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist
import time


class TestBodyController(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = BodyControllerNode()
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.node)

    def tearDown(self):
        self.node.destroy_node()

    def test_joint_state_callback(self):
        """Test that joint state callback processes messages correctly"""
        # Create test message
        msg = JointState()
        msg.name = ['joint1', 'joint2', 'joint3']
        msg.position = [0.1, 0.2, 0.3]
        
        # Call the callback directly to test it
        self.node.joint_state_callback(msg)
        
        # Verify internal state was updated
        self.assertEqual(len(self.node.current_joint_states.name), 3)

    def test_body_command_processing(self):
        """Test body command processing"""
        # Create test command
        cmd = Twist()
        cmd.linear.x = 1.0
        cmd.angular.z = 0.5
        
        # Call the callback
        self.node.body_command_callback(cmd)
        
        # Verify command was stored
        self.assertEqual(self.node.body_command.linear.x, 1.0)
        self.assertEqual(self.node.body_command.angular.z, 0.5)

    def test_node_initialization(self):
        """Test that node initializes correctly"""
        self.assertIsNotNone(self.node.left_arm_cmd_pub)
        self.assertIsNotNone(self.node.right_arm_cmd_pub)
        self.assertIsNotNone(self.node.joint_state_sub)
        self.assertIsNotNone(self.node.body_cmd_sub)


if __name__ == '__main__':
    unittest.main()
```

## Summary

This section has covered essential ROS 2 best practices for humanoid robotics:

1. **Architectural Patterns**: Component-based, layered, and service-oriented architectures for humanoid systems
2. **Communication Patterns**: Proper QoS configuration and efficient message handling
3. **Error Handling**: Fault tolerance and graceful degradation strategies
4. **Performance Optimization**: Memory management and real-time considerations
5. **Security**: Secure communication practices
6. **Testing**: Component testing and validation approaches

These practices ensure that your humanoid robotics applications are robust, maintainable, and performant. Following these guidelines will help you build systems that can handle the complexities and requirements of physical AI applications.

In the next section, we'll apply these concepts with hands-on exercises.