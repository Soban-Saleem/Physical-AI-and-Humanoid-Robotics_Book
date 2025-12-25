# Capstone Exercises: Autonomous Humanoid Project

This section provides hands-on exercises to integrate all modules into a complete autonomous humanoid system. These exercises build upon all previous modules to create a fully functional system that demonstrates the complete Physical AI & Humanoid Robotics textbook concepts.

## Exercise 1: Complete Voice Command to Action Pipeline

### Objective
Create an end-to-end pipeline that receives a voice command, processes it through all systems, and executes a physical action.

### Requirements
- Implement voice command reception and processing
- Use LLM for task decomposition
- Integrate with navigation and manipulation systems
- Execute complete behavior in simulation
- Validate safety before action execution

### Steps to Complete
- Create a voice command node that captures and transcribes voice input
- Connect to the cognitive planning system to decompose the command
- Integrate with navigation and manipulation systems
- Implement safety validation before execution
- Test with a complete "fetch and carry" command

### Solution Approach

```python
# voice_to_action_pipeline.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist, Pose
from sensor_msgs.msg import Image, LaserScan
import json
import asyncio

class VoiceToActionPipeline(Node):
    def __init__(self):
        super().__init__('voice_to_action_pipeline')
        
        # Publishers for different system components
        self.voice_cmd_publisher = self.create_publisher(
            String, '/voice_commands', 10)
        self.nav_goal_publisher = self.create_publisher(
            Pose, '/navigation/goal', 10)
        self.manip_cmd_publisher = self.create_publisher(
            String, '/manipulation_commands', 10)
        self.base_cmd_publisher = self.create_publisher(
            Twist, '/cmd_vel', 10)
        
        # Subscriptions for monitoring
        self.status_subscription = self.create_subscription(
            String, '/system_status', self.status_callback, 10)
        self.nav_status_subscription = self.create_subscription(
            String, '/navigation/status', self.nav_status_callback, 10)
        self.manip_status_subscription = self.create_subscription(
            String, '/manipulation/status', self.manip_status_callback, 10)
        
        # System state tracking
        self.system_state = 'idle'
        self.navigation_active = False
        self.manipulation_active = False
        
        # Task queue for handling multiple commands
        self.task_queue = []
        self.current_task = None
        
        self.get_logger().info('Voice-to-Action Pipeline initialized')

    def status_callback(self, msg):
        """Update system status"""
        try:
            status_data = json.loads(msg.data)
            self.system_state = status_data.get('state', 'unknown')
        except json.JSONDecodeError:
            self.system_state = msg.data

    def nav_status_callback(self, msg):
        """Update navigation status"""
        self.navigation_active = 'ACTIVE' in msg.data.upper()

    def manip_status_callback(self, msg):
        """Update manipulation status"""
        self.manipulation_active = 'ACTIVE' in msg.data.upper()

    def process_voice_command(self, command_text):
        """Process voice command through entire pipeline"""
        self.get_logger().info(f'Processing voice command: {command_text}')
        
        # Update system status
        status_msg = String()
        status_msg.data = json.dumps({
            'state': 'processing_command',
            'command': command_text,
            'timestamp': self.get_clock().now().nanoseconds
        })
        self.status_publisher.publish(status_msg)
        
        # Plan the action sequence using cognitive planning
        action_sequence = self.plan_action_sequence(command_text)
        
        if action_sequence:
            # Validate the action sequence for safety
            if self.validate_action_sequence(action_sequence):
                # Execute the sequence
                self.execute_action_sequence(action_sequence)
            else:
                self.get_logger().error('Action sequence failed safety validation')
        else:
            self.get_logger().error(f'Could not plan action sequence for: {command_text}')

    def plan_action_sequence(self, command):
        """Plan sequence of actions for the given command"""
        # In a real implementation, this would connect to the LLM cognitive planner
        # For this example, we'll implement a simple rule-based planner
        
        command_lower = command.lower()
        
        if 'go to' in command_lower or 'navigate to' in command_lower:
            # Extract destination
            destination = self.extract_destination(command)
            return [
                {
                    'action_type': 'navigate_to_location',
                    'parameters': {'location': destination},
                    'description': f'Navigate to {destination}'
                }
            ]
        elif 'pick up' in command_lower or 'get' in command_lower:
            # Extract object
            obj = self.extract_object(command)
            return [
                {
                    'action_type': 'find_object',
                    'parameters': {'object': obj},
                    'description': f'Locate {obj}'
                },
                {
                    'action_type': 'navigate_to_object',
                    'parameters': {'object': obj},
                    'description': f'Navigate to {obj}'
                },
                {
                    'action_type': 'grasp_object',
                    'parameters': {'object': obj},
                    'description': f'Grasp {obj}'
                }
            ]
        elif 'bring' in command_lower or 'fetch' in command_lower:
            # Extract object and destination
            obj = self.extract_object(command)
            dest = self.extract_destination(command)
            return [
                {
                    'action_type': 'find_object',
                    'parameters': {'object': obj},
                    'description': f'Locate {obj}'
                },
                {
                    'action_type': 'navigate_to_object',
                    'parameters': {'object': obj},
                    'description': f'Navigate to {obj}'
                },
                {
                    'action_type': 'grasp_object',
                    'parameters': {'object': obj},
                    'description': f'Grasp {obj}'
                },
                {
                    'action_type': 'navigate_to_location',
                    'parameters': {'location': dest},
                    'description': f'Navigate to {dest}'
                },
                {
                    'action_type': 'release_object',
                    'parameters': {'location': dest},
                    'description': f'Release object at {dest}'
                }
            ]
        else:
            return None

    def extract_destination(self, command):
        """Extract destination from command (simplified implementation)"""
        # Look for location after 'to'
        words = command.lower().split()
        for i, word in enumerate(words):
            if word == 'to' and i+1 < len(words):
                return words[i+1]
        return 'destination'

    def extract_object(self, command):
        """Extract object from command (simplified implementation)"""
        # Look for object after 'pick up', 'get', 'bring', etc.
        words = command.lower().split()
        for i, word in enumerate(words):
            if word in ['the', 'a', 'an'] and i+1 < len(words):
                return words[i+1]
        return 'object'

    def validate_action_sequence(self, sequence):
        """Validate action sequence for safety"""
        for action in sequence:
            action_type = action['action_type']
            
            # Check for potential safety issues
            if action_type == 'navigate_to_location':
                # Verify destination is safe and reachable
                destination = action['parameters'].get('location', '')
                if not self.is_safe_destination(destination):
                    self.get_logger().error(f'Destination {destination} is not safe')
                    return False
            elif action_type == 'grasp_object':
                # Verify object is graspable
                obj = action['parameters'].get('object', '')
                if not self.is_graspable_object(obj):
                    self.get_logger().error(f'Object {obj} is not graspable')
                    return False
        
        return True

    def execute_action_sequence(self, sequence):
        """Execute sequence of actions"""
        self.get_logger().info(f'Executing action sequence with {len(sequence)} actions')
        
        # Execute actions sequentially
        for i, action in enumerate(sequence):
            self.get_logger().info(f'Executing action {i+1}/{len(sequence)}: {action["description"]}')
            
            success = self.execute_single_action(action)
            if not success:
                self.get_logger().error(f'Action failed: {action["description"]}')
                break
        
        # Update system status
        status_msg = String()
        status_msg.data = json.dumps({
            'state': 'idle',
            'last_task': 'completed' if i == len(sequence)-1 else 'failed',
            'timestamp': self.get_clock().now().nanoseconds
        })
        self.status_publisher.publish(status_msg)

    def execute_single_action(self, action):
        """Execute a single action"""
        action_type = action['action_type']
        
        if action_type == 'navigate_to_location':
            return self.execute_navigation_action(action)
        elif action_type == 'find_object':
            return self.execute_find_object_action(action)
        elif action_type == 'navigate_to_object':
            return self.execute_navigate_to_object_action(action)
        elif action_type == 'grasp_object':
            return self.execute_grasp_action(action)
        elif action_type == 'release_object':
            return self.execute_release_action(action)
        else:
            self.get_logger().error(f'Unknown action type: {action_type}')
            return False

    def execute_navigation_action(self, action):
        """Execute navigation action"""
        # Create navigation goal
        goal = Pose()
        # In a real implementation, this would look up the location coordinates
        goal.position.x = 1.0  # Placeholder coordinates
        goal.position.y = 1.0
        goal.position.z = 0.0
        goal.orientation.w = 1.0  # Identity quaternion
        
        self.nav_goal_publisher.publish(goal)
        self.get_logger().info(f'Published navigation goal to {action["parameters"]["location"]}')
        return True

    def execute_find_object_action(self, action):
        """Execute object finding action"""
        # In a real implementation, this would activate computer vision systems
        obj_name = action['parameters']['object']
        self.get_logger().info(f'Looking for object: {obj_name}')
        # This would typically involve activating camera systems and object detection
        return True

    def execute_navigate_to_object_action(self, action):
        """Execute navigation to object action"""
        obj_name = action['parameters']['object']
        self.get_logger().info(f'Navigating to {obj_name}')
        # This would involve using detected object position as navigation goal
        return True

    def execute_grasp_action(self, action):
        """Execute grasping action"""
        obj_name = action['parameters']['object']
        self.get_logger().info(f'Attempting to grasp {obj_name}')
        # This would involve activating manipulation system
        manip_cmd = String()
        manip_cmd.data = json.dumps({
            'action': 'grasp',
            'object': obj_name,
            'timestamp': self.get_clock().now().nanoseconds
        })
        self.manip_cmd_publisher.publish(manip_cmd)
        return True

    def execute_release_action(self, action):
        """Execute release action"""
        location = action['parameters'].get('location', 'current')
        self.get_logger().info(f'Releasing object at {location}')
        # This would involve activating manipulation system to release
        manip_cmd = String()
        manip_cmd.data = json.dumps({
            'action': 'release',
            'location': location,
            'timestamp': self.get_clock().now().nanoseconds
        })
        self.manip_cmd_publisher.publish(manip_cmd)
        return True

    def is_safe_destination(self, destination):
        """Check if destination is safe (simplified implementation)"""
        # In a real implementation, this would check:
        # - Destination is in known map
        # - Path to destination is clear of obstacles
        # - Destination is not in restricted area
        return True  # Placeholder - assume all destinations are safe

    def is_graspable_object(self, obj_name):
        """Check if object is graspable (simplified implementation)"""
        # In a real implementation, this would check:
        # - Object exists and is detected
        # - Object is within reach
        # - Object size and weight are graspable
        return True  # Placeholder - assume all objects are graspable

def main(args=None):
    rclpy.init(args=args)
    pipeline = VoiceToActionPipeline()
    
    # Example: Send a command to test the pipeline
    def send_test_command():
        test_cmd = String()
        test_cmd.data = "bring me the red cup from the table"
        pipeline.voice_cmd_publisher.publish(test_cmd)
        pipeline.get_logger().info('Sent test command: "bring me the red cup from the table"')
    
    # Send test command after 5 seconds
    pipeline.create_timer(5.0, send_test_command)
    
    try:
        rclpy.spin(pipeline)
    except KeyboardInterrupt:
        pipeline.get_logger().info('Shutting down voice-to-action pipeline...')
    finally:
        pipeline.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Exercise 2: Multi-Modal Perception Integration

### Objective
Integrate multiple perception modalities (vision, LiDAR, IMU) to create a comprehensive understanding of the environment.

### Requirements
- Combine data from multiple sensors for environment understanding
- Implement sensor fusion techniques
- Create unified perception output
- Handle sensor failures gracefully
- Validate perception accuracy

### Steps to Complete
- Create a sensor fusion node that subscribes to multiple sensor topics
- Implement data association and fusion algorithms
- Create a unified representation of the environment
- Handle sensor failures and missing data
- Validate the fused perception output

### Solution Approach

```python
# sensor_fusion_node.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, LaserScan, Imu
from geometry_msgs.msg import PointStamped
from std_msgs.msg import String
from vision_msgs.msg import Detection2DArray
import numpy as np
import json

class MultiModalPerceptionFusion(Node):
    def __init__(self):
        super().__init__('multi_modal_perception_fusion')
        
        # Subscriptions for different sensors
        self.camera_subscription = self.create_subscription(
            Image, '/camera/image_rect_color', self.camera_callback, 10)
        self.lidar_subscription = self.create_subscription(
            LaserScan, '/scan', self.lidar_callback, 10)
        self.imu_subscription = self.create_subscription(
            Imu, '/imu/data', self.imu_callback, 10)
        self.detection_subscription = self.create_subscription(
            Detection2DArray, '/object_detections', self.detection_callback, 10)
        
        # Publisher for fused perception
        self.fused_perception_publisher = self.create_publisher(
            String, '/fused_perception', 10)
        
        # Storage for sensor data
        self.latest_camera = None
        self.latest_lidar = None
        self.latest_imu = None
        self.latest_detections = None
        
        # TF buffer for coordinate transformations
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # Timer for fusion processing
        self.fusion_timer = self.create_timer(0.1, self.perform_sensor_fusion)
        
        self.get_logger().info('Multi-Modal Perception Fusion initialized')

    def camera_callback(self, msg):
        """Process camera data"""
        self.latest_camera = msg
        self.last_camera_time = self.get_clock().now()

    def lidar_callback(self, msg):
        """Process LiDAR data"""
        self.latest_lidar = msg
        self.last_lidar_time = self.get_clock().now()

    def imu_callback(self, msg):
        """Process IMU data"""
        self.latest_imu = msg
        self.last_imu_time = self.get_clock().now()

    def detection_callback(self, msg):
        """Process object detection data"""
        self.latest_detections = msg
        self.last_detection_time = self.get_clock().now()

    def perform_sensor_fusion(self):
        """Perform sensor fusion when all sensors have updated"""
        # Check if we have data from all sensors
        if not all([self.latest_camera, self.latest_lidar, self.latest_imu, self.latest_detections]):
            return  # Wait for all sensors to have data
        
        # Check timing - ensure data is not too stale
        current_time = self.get_clock().now()
        if (current_time - self.last_camera_time).nanoseconds > 1e9 or \
           (current_time - self.last_lidar_time).nanoseconds > 1e9 or \
           (current_time - self.last_imu_time).nanoseconds > 1e9 or \
           (current_time - self.last_detection_time).nanoseconds > 1e9:
            self.get_logger().warn('Sensor data is too stale for fusion')
            return
        
        # Perform sensor fusion
        fused_data = self.fuse_sensor_data()
        
        if fused_data:
            # Publish fused perception data
            fused_msg = String()
            fused_msg.data = json.dumps(fused_data)
            self.fused_perception_publisher.publish(fused_msg)
            
            self.get_logger().info(f'Published fused perception with {len(fused_data.get("objects", []))} objects')

    def fuse_sensor_data(self):
        """Fuse data from multiple sensors"""
        try:
            fused_result = {
                'timestamp': self.get_clock().now().nanoseconds,
                'objects': [],
                'environment_map': [],
                'robot_state': {
                    'position': self.estimate_robot_position(),
                    'orientation': self.estimate_robot_orientation(),
                    'velocity': self.estimate_robot_velocity()
                }
            }
            
            # Fuse object detections from camera with LiDAR data
            if self.latest_detections and self.latest_lidar:
                objects = self.correlate_vision_lidar_data()
                fused_result['objects'] = objects
            
            # Add environment map based on LiDAR
            if self.latest_lidar:
                env_map = self.create_environment_map()
                fused_result['environment_map'] = env_map
            
            return fused_result
            
        except Exception as e:
            self.get_logger().error(f'Error in sensor fusion: {e}')
            return None

    def correlate_vision_lidar_data(self):
        """Correlate vision and LiDAR data to create unified object representations"""
        # This is a simplified implementation
        # In reality, this would involve complex algorithms to associate
        # 2D image detections with 3D LiDAR points
        
        correlated_objects = []
        
        if self.latest_detections:
            for detection in self.latest_detections.detections:
                if detection.results:  # If there are detection results
                    best_result = max(detection.results, key=lambda x: x.score)
                    
                    # Create object with detection info
                    obj = {
                        'id': best_result.id,
                        'confidence': best_result.score,
                        'bbox_2d': {
                            'center_x': detection.bbox.center.x,
                            'center_y': detection.bbox.center.y,
                            'size_x': detection.bbox.size_x,
                            'size_y': detection.bbox.size_y
                        },
                        'estimated_3d_position': self.estimate_3d_position(
                            detection.bbox.center.x, 
                            detection.bbox.center.y
                        )
                    }
                    correlated_objects.append(obj)
        
        return correlated_objects

    def estimate_3d_position(self, pixel_x, pixel_y):
        """Estimate 3D position from 2D pixel and depth information"""
        # In a real implementation, this would use camera intrinsics and depth data
        # For this example, we'll return a placeholder
        return {
            'x': float(pixel_x) / 100.0,  # Rough estimation
            'y': float(pixel_y) / 100.0,
            'z': 1.0  # Fixed height for simplicity
        }

    def create_environment_map(self):
        """Create environment map from LiDAR data"""
        # Create a simple occupancy grid from LiDAR data
        if not self.latest_lidar:
            return []
        
        # Simplified occupancy grid creation
        grid_resolution = 0.1  # 10cm per cell
        grid_size = 20  # 20m x 20m grid
        grid_origin = (-grid_size/2, -grid_size/2)
        
        occupancy_grid = []
        
        # Process LiDAR ranges to create occupancy information
        for i, range_val in enumerate(self.latest_lidar.ranges):
            if range_val < self.latest_lidar.range_max and range_val > self.latest_lidar.range_min:
                # Convert polar to Cartesian
                angle = self.latest_lidar.angle_min + i * self.latest_lidar.angle_increment
                x = range_val * np.cos(angle)
                y = range_val * np.sin(angle)
                
                # Convert to grid coordinates
                grid_x = int((x - grid_origin[0]) / grid_resolution)
                grid_y = int((y - grid_origin[1]) / grid_resolution)
                
                if 0 <= grid_x < grid_size and 0 <= grid_y < grid_size:
                    occupancy_grid.append({
                        'x': grid_x,
                        'y': grid_y,
                        'occupancy': 1.0,  # Occupied
                        'distance': range_val
                    })
        
        return occupancy_grid

    def estimate_robot_position(self):
        """Estimate robot position using sensor data"""
        # In a real implementation, this would use sensor fusion
        # like Kalman filters or particle filters
        # For this example, return a placeholder
        return {'x': 0.0, 'y': 0.0, 'z': 0.0}

    def estimate_robot_orientation(self):
        """Estimate robot orientation using IMU data"""
        if self.latest_imu:
            return {
                'x': self.latest_imu.orientation.x,
                'y': self.latest_imu.orientation.y,
                'z': self.latest_imu.orientation.z,
                'w': self.latest_imu.orientation.w
            }
        return {'x': 0.0, 'y': 0.0, 'z': 0.0, 'w': 1.0}

    def estimate_robot_velocity(self):
        """Estimate robot velocity using IMU and LiDAR data"""
        # In a real implementation, this would use more sophisticated fusion
        if self.latest_imu:
            return {
                'linear': {
                    'x': self.latest_imu.linear_acceleration.x,
                    'y': self.latest_imu.linear_acceleration.y,
                    'z': self.latest_imu.linear_acceleration.z
                },
                'angular': {
                    'x': self.latest_imu.angular_velocity.x,
                    'y': self.latest_imu.angular_velocity.y,
                    'z': self.latest_imu.angular_velocity.z
                }
            }
        return {'linear': {'x': 0.0, 'y': 0.0, 'z': 0.0}, 'angular': {'x': 0.0, 'y': 0.0, 'z': 0.0}}

def main(args=None):
    rclpy.init(args=args)
    fusion_node = MultiModalPerceptionFusion()
    
    try:
        rclpy.spin(fusion_node)
    except KeyboardInterrupt:
        fusion_node.get_logger().info('Shutting down sensor fusion node...')
    finally:
        fusion_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Exercise 3: Humanoid-Specific Behavior Implementation

### Objective
Implement a humanoid-specific behavior that demonstrates bipedal locomotion, manipulation, and human-like interaction.

### Requirements
- Implement bipedal walking pattern
- Create manipulation behavior for humanoid hands
- Implement human-aware navigation (considering human presence)
- Design behavior that demonstrates human-like interaction
- Validate safety of humanoid-specific movements

### Steps to Complete
- Create a humanoid locomotion controller
- Implement manipulation behaviors for humanoid hands
- Design human-aware navigation system
- Create human-like interaction patterns
- Validate all behaviors for safety and feasibility

### Solution Approach

```python
# humanoid_behavior_controller.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist, Pose
from sensor_msgs.msg import JointState
from builtin_interfaces.msg import Duration
import numpy as np
import math

class HumanoidBehaviorController(Node):
    def __init__(self):
        super().__init__('humanoid_behavior_controller')
        
        # Publishers for different control systems
        self.joint_cmd_publisher = self.create_publisher(
            JointState, '/joint_commands', 10)
        self.base_cmd_publisher = self.create_publisher(
            Twist, '/cmd_vel', 10)
        self.behavior_status_publisher = self.create_publisher(
            String, '/behavior_status', 10)
        
        # Subscriptions for sensor data
        self.joint_state_subscription = self.create_subscription(
            JointState, '/joint_states', self.joint_state_callback, 10)
        self.imu_subscription = self.create_subscription(
            Imu, '/imu/data', self.imu_callback, 10)
        
        # Behavior control
        self.current_behavior = 'idle'
        self.behavior_timer = self.create_timer(0.05, self.execute_behavior)  # 20Hz control loop
        
        # Joint state storage
        self.current_joint_states = JointState()
        self.imu_data = None
        
        # Humanoid-specific parameters
        self.step_height = 0.05  # 5cm step height
        self.step_length = 0.2   # 20cm step length
        self.step_duration = 1.0  # 1 second per step
        self.hip_height = 0.6    # Height of hip from ground
        
        self.get_logger().info('Humanoid Behavior Controller initialized')

    def joint_state_callback(self, msg):
        """Update current joint states"""
        self.current_joint_states = msg

    def imu_callback(self, msg):
        """Update IMU data for balance control"""
        self.imu_data = msg

    def execute_behavior(self):
        """Execute current humanoid behavior"""
        if self.current_behavior == 'walking':
            self.execute_walking_behavior()
        elif self.current_behavior == 'balancing':
            self.execute_balance_behavior()
        elif self.current_behavior == 'manipulating':
            self.execute_manipulation_behavior()
        elif self.current_behavior == 'idle':
            self.execute_idle_behavior()
        else:
            self.execute_idle_behavior()

    def execute_walking_behavior(self):
        """Execute bipedal walking pattern"""
        # Implement simplified walking gait
        # This is a simplified approach - real implementation would use
        # more sophisticated inverse kinematics and balance control
        
        current_time = self.get_clock().now().nanoseconds / 1e9
        
        # Generate walking pattern based on time
        phase = (current_time / self.step_duration) % 2.0
        swing_leg = 'left' if phase < 1.0 else 'right'
        
        # Calculate joint angles for walking gait
        joint_angles = self.calculate_walking_gait(phase, swing_leg)
        
        # Create joint command message
        joint_cmd = JointState()
        joint_cmd.header.stamp = self.get_clock().now().to_msg()
        joint_cmd.name = list(joint_angles.keys())
        joint_cmd.position = list(joint_angles.values())
        
        self.joint_cmd_publisher.publish(joint_cmd)
        
        # Publish base velocity for forward motion
        base_cmd = Twist()
        base_cmd.linear.x = 0.3  # Forward velocity
        self.base_cmd_publisher.publish(base_cmd)

    def calculate_walking_gait(self, phase, swing_leg):
        """Calculate joint angles for walking gait"""
        # Simplified walking gait - in reality, this would use inverse kinematics
        # and more sophisticated gait planning
        
        # Oscillate between different joint configurations for walking
        t = phase  # 0-2 for complete gait cycle
        
        # Hip joints
        left_hip_roll = 0.1 * math.sin(2 * math.pi * t) if swing_leg == 'left' else 0
        right_hip_roll = 0.1 * math.sin(2 * math.pi * t) if swing_leg == 'right' else 0
        left_hip_pitch = 0.2 * math.sin(math.pi * t) if swing_leg == 'left' else 0
        right_hip_pitch = 0.2 * math.sin(math.pi * t) if swing_leg == 'right' else 0
        
        # Knee joints
        left_knee = 0.3 * math.sin(math.pi * t) if swing_leg == 'left' else 0
        right_knee = 0.3 * math.sin(math.pi * t) if swing_leg == 'right' else 0
        
        # Ankle joints
        left_ankle = -0.2 * math.sin(math.pi * t) if swing_leg == 'left' else 0
        right_ankle = -0.2 * math.sin(math.pi * t) if swing_leg == 'right' else 0
        
        # Return joint angles (names and values would match robot URDF)
        joint_angles = {
            'left_hip_roll': left_hip_roll,
            'left_hip_yaw': 0.0,
            'left_hip_pitch': left_hip_pitch,
            'left_knee': left_knee,
            'left_ankle_pitch': left_ankle,
            'left_ankle_roll': 0.0,
            'right_hip_roll': right_hip_roll,
            'right_hip_yaw': 0.0,
            'right_hip_pitch': right_hip_pitch,
            'right_knee': right_knee,
            'right_ankle_pitch': right_ankle,
            'right_ankle_roll': 0.0,
            # Add more joints as needed
        }
        
        return joint_angles

    def execute_balance_behavior(self):
        """Execute balance control behavior"""
        # Use IMU data to maintain balance
        if self.imu_data:
            # Simple balance control based on IMU readings
            roll = self.imu_data.orientation.x
            pitch = self.imu_data.orientation.y
            
            # Calculate corrective joint angles based on orientation error
            corrective_angles = {
                'left_ankle_roll': -roll * 0.5,
                'left_ankle_pitch': -pitch * 0.5,
                'right_ankle_roll': -roll * 0.5,
                'right_ankle_pitch': -pitch * 0.5,
                # Hip adjustments for larger corrections
                'left_hip_roll': -roll * 0.2,
                'left_hip_pitch': -pitch * 0.2,
                'right_hip_roll': -roll * 0.2,
                'right_hip_pitch': -pitch * 0.2,
            }
            
            # Publish corrective joint commands
            joint_cmd = JointState()
            joint_cmd.header.stamp = self.get_clock().now().to_msg()
            joint_cmd.name = list(corrective_angles.keys())
            joint_cmd.position = list(corrective_angles.values())
            
            self.joint_cmd_publisher.publish(joint_cmd)

    def execute_manipulation_behavior(self):
        """Execute manipulation behavior for humanoid hands"""
        # For this example, implement a simple reaching/grasping behavior
        # In reality, this would involve complex inverse kinematics and grasp planning
        
        # Example: Reach forward and grasp
        joint_angles = {
            # Left arm
            'left_shoulder_pan': 0.0,
            'left_shoulder_lift': -0.5,
            'left_elbow_flex': 1.0,
            'left_wrist_flex': 0.0,
            'left_wrist_roll': 0.0,
            # Right arm
            'right_shoulder_pan': 0.0,
            'right_shoulder_lift': -0.5,
            'right_elbow_flex': 1.0,
            'right_wrist_flex': 0.0,
            'right_wrist_roll': 0.0,
            # Hand joints (simplified)
            'left_hand_thumb': 0.0,
            'left_hand_index': 0.0,
            'left_hand_middle': 0.0,
            'left_hand_ring': 0.0,
            'left_hand_pinky': 0.0,
            'right_hand_thumb': 0.0,
            'right_hand_index': 0.0,
            'right_hand_middle': 0.0,
            'right_hand_ring': 0.0,
            'right_hand_pinky': 0.0,
        }
        
        # Publish manipulation joint commands
        joint_cmd = JointState()
        joint_cmd.header.stamp = self.get_clock().now().to_msg()
        joint_cmd.name = list(joint_angles.keys())
        joint_cmd.position = list(joint_angles.values())
        
        self.joint_cmd_publisher.publish(joint_cmd)

    def execute_idle_behavior(self):
        """Execute idle behavior (stand in neutral position)"""
        # Publish neutral joint angles
        neutral_angles = {
            'left_hip_roll': 0.0,
            'left_hip_yaw': 0.0,
            'left_hip_pitch': 0.0,
            'left_knee': 0.0,
            'left_ankle_pitch': 0.0,
            'left_ankle_roll': 0.0,
            'right_hip_roll': 0.0,
            'right_hip_yaw': 0.0,
            'right_hip_pitch': 0.0,
            'right_knee': 0.0,
            'right_ankle_pitch': 0.0,
            'right_ankle_roll': 0.0,
            # Add all other joints in neutral position
        }
        
        joint_cmd = JointState()
        joint_cmd.header.stamp = self.get_clock().now().to_msg()
        joint_cmd.name = list(neutral_angles.keys())
        joint_cmd.position = list(neutral_angles.values())
        
        self.joint_cmd_publisher.publish(joint_cmd)

    def start_behavior(self, behavior_name):
        """Start a specific humanoid behavior"""
        if behavior_name in ['walking', 'balancing', 'manipulating', 'idle']:
            self.current_behavior = behavior_name
            status_msg = String()
            status_msg.data = f"BEHAVIOR_STARTED: {behavior_name}"
            self.behavior_status_publisher.publish(status_msg)
            self.get_logger().info(f'Started humanoid behavior: {behavior_name}')
        else:
            self.get_logger().error(f'Unknown behavior: {behavior_name}')

    def stop_behavior(self):
        """Stop current behavior and return to idle"""
        self.current_behavior = 'idle'
        status_msg = String()
        status_msg.data = "BEHAVIOR_STOPPED"
        self.behavior_status_publisher.publish(status_msg)
        self.get_logger().info('Stopped humanoid behavior, returned to idle')

def main(args=None):
    rclpy.init(args=args)
    controller = HumanoidBehaviorController()
    
    # Example: Start walking behavior after 5 seconds
    def start_walking():
        controller.start_behavior('walking')
    
    controller.create_timer(5.0, start_walking)
    
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        controller.get_logger().info('Shutting down humanoid behavior controller...')
    finally:
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Exercise 4: System Integration and Validation

### Objective
Integrate all components into a complete autonomous humanoid system and validate its functionality.

### Requirements
- Connect all modules (ROS 2, simulation, AI perception, VLA) into one system
- Implement system state management
- Create validation procedures for the complete system
- Test system behavior in complex scenarios
- Document system performance and limitations

### Steps to Complete
- Create a system coordinator that manages all components
- Implement state machine for system operation
- Design validation procedures for the integrated system
- Test with complex multi-step scenarios
- Document system performance metrics

### Solution Approach

```python
# system_coordinator.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist, Pose
from sensor_msgs.msg import Image, LaserScan
from vision_msgs.msg import Detection2DArray
import json
import time
from enum import Enum

class SystemState(Enum):
    IDLE = "idle"
    PROCESSING_COMMAND = "processing_command"
    NAVIGATING = "navigating"
    MANIPULATING = "manipulating"
    PERFORMING_TASK = "performing_task"
    ERROR = "error"
    EMERGENCY_STOP = "emergency_stop"

class SystemCoordinator(Node):
    def __init__(self):
        super().__init__('system_coordinator')
        
        # Subscriptions for all system components
        self.voice_cmd_subscription = self.create_subscription(
            String, '/voice_commands', self.voice_command_callback, 10)
        self.perception_subscription = self.create_subscription(
            String, '/fused_perception', self.perception_callback, 10)
        self.nav_status_subscription = self.create_subscription(
            String, '/navigation/status', self.nav_status_callback, 10)
        self.manip_status_subscription = self.create_subscription(
            String, '/manipulation/status', self.manip_status_callback, 10)
        self.safety_subscription = self.create_subscription(
            String, '/safety_status', self.safety_callback, 10)
        
        # Publishers for system control
        self.system_cmd_publisher = self.create_publisher(
            String, '/system_commands', 10)
        self.status_publisher = self.create_publisher(
            String, '/system_status', 10)
        self.emergency_publisher = self.create_publisher(
            String, '/emergency_stop', 10)
        
        # System state management
        self.current_state = SystemState.IDLE
        self.previous_state = SystemState.IDLE
        self.active_task = None
        self.system_health = True
        self.emergency_stop_active = False
        
        # System validation metrics
        self.performance_metrics = {
            'command_response_time': [],
            'task_success_rate': 0.0,
            'error_count': 0,
            'uptime': 0.0
        }
        
        # Timer for system monitoring
        self.monitor_timer = self.create_timer(1.0, self.system_monitor)
        
        self.get_logger().info('System Coordinator initialized')
        self.publish_system_status()

    def voice_command_callback(self, msg):
        """Process voice commands and initiate appropriate system behavior"""
        if self.emergency_stop_active:
            self.get_logger().warn('Emergency stop active, ignoring commands')
            return
            
        if self.current_state != SystemState.IDLE:
            self.get_logger().warn(f'System busy in state {self.current_state.value}, queuing command')
            # In a real implementation, add to command queue
            return
        
        try:
            command_data = json.loads(msg.data)
            command_text = command_data.get('command', '')
            
            self.get_logger().info(f'Received voice command: {command_text}')
            
            # Transition to processing state
            self.transition_to_state(SystemState.PROCESSING_COMMAND)
            
            # Plan and execute the command
            success = self.execute_voice_command(command_text)
            
            if success:
                self.transition_to_state(SystemState.IDLE)
                self.get_logger().info('Command completed successfully')
            else:
                self.transition_to_state(SystemState.ERROR)
                self.get_logger().error('Command execution failed')
                
        except json.JSONDecodeError:
            self.get_logger().error(f'Invalid JSON in voice command: {msg.data}')
            self.transition_to_state(SystemState.ERROR)
        except Exception as e:
            self.get_logger().error(f'Error processing voice command: {e}')
            self.transition_to_state(SystemState.ERROR)

    def execute_voice_command(self, command_text):
        """Execute a voice command by coordinating all system components"""
        # In a real implementation, this would connect to the cognitive planning system
        # For this example, we'll implement a simple command processor
        
        command_lower = command_text.lower()
        
        if 'walk' in command_lower or 'move' in command_lower:
            return self.execute_navigation_task(command_text)
        elif 'pick' in command_lower or 'grasp' in command_lower:
            return self.execute_manipulation_task(command_text)
        elif 'navigate' in command_lower or 'go to' in command_lower:
            return self.execute_navigation_task(command_text)
        else:
            self.get_logger().warn(f'Unknown command type: {command_text}')
            return False

    def execute_navigation_task(self, command):
        """Execute navigation task"""
        self.get_logger().info(f'Executing navigation task: {command}')
        self.transition_to_state(SystemState.NAVIGATING)
        
        # In a real implementation, this would:
        # 1. Use cognitive planning to decompose the task
        # 2. Publish navigation goals
        # 3. Monitor navigation progress
        # 4. Handle navigation errors
        
        # For this example, we'll simulate navigation
        goal_pose = Pose()
        goal_pose.position.x = 1.0
        goal_pose.position.y = 1.0
        goal_pose.position.z = 0.0
        goal_pose.orientation.w = 1.0
        
        # Publish navigation goal
        nav_goal_publisher = self.create_publisher(Pose, '/navigation/goal', 10)
        nav_goal_publisher.publish(goal_pose)
        
        # Simulate navigation completion after 5 seconds
        start_time = time.time()
        while time.time() - start_time < 5.0 and self.current_state == SystemState.NAVIGATING:
            time.sleep(0.1)
            rclpy.spin_once(self, timeout_sec=0.1)
        
        return True  # Simulated success

    def execute_manipulation_task(self, command):
        """Execute manipulation task"""
        self.get_logger().info(f'Executing manipulation task: {command}')
        self.transition_to_state(SystemState.MANIPULATING)
        
        # In a real implementation, this would:
        # 1. Use perception to locate the object
        # 2. Plan grasp trajectory
        # 3. Execute manipulation
        # 4. Verify success
        
        # For this example, we'll simulate manipulation
        manip_cmd_publisher = self.create_publisher(String, '/manipulation/commands', 10)
        
        manip_cmd = String()
        manip_cmd.data = json.dumps({
            'action': 'grasp',
            'object': 'unknown_object',
            'timestamp': self.get_clock().now().nanoseconds
        })
        manip_cmd_publisher.publish(manip_cmd)
        
        # Simulate manipulation completion after 3 seconds
        start_time = time.time()
        while time.time() - start_time < 3.0 and self.current_state == SystemState.MANIPULATING:
            time.sleep(0.1)
            rclpy.spin_once(self, timeout_sec=0.1)
        
        return True  # Simulated success

    def perception_callback(self, msg):
        """Process fused perception data"""
        try:
            perception_data = json.loads(msg.data)
            # Store perception data for system use
            self.latest_perception = perception_data
        except json.JSONDecodeError:
            self.get_logger().warn('Invalid JSON in perception data')

    def nav_status_callback(self, msg):
        """Process navigation status updates"""
        # Handle navigation state changes
        if "SUCCESS" in msg.data.upper():
            if self.current_state == SystemState.NAVIGATING:
                self.transition_to_next_state()
        elif "FAILED" in msg.data.upper():
            self.transition_to_state(SystemState.ERROR)

    def manip_status_callback(self, msg):
        """Process manipulation status updates"""
        # Handle manipulation state changes
        if "SUCCESS" in msg.data.upper():
            if self.current_state == SystemState.MANIPULATING:
                self.transition_to_next_state()
        elif "FAILED" in msg.data.upper():
            self.transition_to_state(SystemState.ERROR)

    def safety_callback(self, msg):
        """Process safety status updates"""
        if "EMERGENCY" in msg.data.upper():
            self.emergency_stop_active = True
            self.transition_to_state(SystemState.EMERGENCY_STOP)
        elif "NORMAL" in msg.data.upper():
            self.emergency_stop_active = False

    def transition_to_state(self, new_state):
        """Safely transition the system to a new state"""
        if self.current_state != new_state:
            self.get_logger().info(f'System state transition: {self.current_state.value} → {new_state.value}')
            self.previous_state = self.current_state
            self.current_state = new_state
            self.publish_system_status()

    def transition_to_next_state(self):
        """Transition to the appropriate next state based on context"""
        if self.current_state == SystemState.NAVIGATING:
            self.transition_to_state(SystemState.IDLE)
        elif self.current_state == SystemState.MANIPULATING:
            self.transition_to_state(SystemState.IDLE)
        else:
            self.transition_to_state(SystemState.IDLE)

    def system_monitor(self):
        """Monitor system health and performance"""
        # Check for system issues
        if not self.system_health:
            self.get_logger().error('System health issue detected')
            self.transition_to_state(SystemState.ERROR)
        
        # Publish system status periodically
        self.publish_system_status()

    def publish_system_status(self):
        """Publish current system status"""
        status_msg = String()
        status_msg.data = json.dumps({
            'current_state': self.current_state.value,
            'previous_state': self.previous_state.value,
            'system_health': self.system_health,
            'emergency_stop': self.emergency_stop_active,
            'timestamp': self.get_clock().now().nanoseconds,
            'performance_metrics': self.performance_metrics
        })
        self.status_publisher.publish(status_msg)

    def reset_system(self):
        """Reset system to initial state"""
        self.transition_to_state(SystemState.IDLE)
        self.active_task = None
        self.system_health = True
        self.emergency_stop_active = False
        self.get_logger().info('System reset to initial state')

def main(args=None):
    rclpy.init(args=args)
    coordinator = SystemCoordinator()
    
    try:
        rclpy.spin(coordinator)
    except KeyboardInterrupt:
        coordinator.get_logger().info('Shutting down system coordinator...')
    finally:
        coordinator.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Self-Assessment Questions

- How would you validate that the voice command pipeline properly decomposes complex tasks into executable actions?
- What metrics would you use to measure the quality of sensor fusion in your system?
- How would you ensure humanoid walking patterns maintain stability and balance?
- What safety checks would you implement before executing any physical action?
- How would you handle the transition from simulation to real hardware?

## Advanced Challenges (Optional)

- **Multi-Robot Coordination**: Extend the system to coordinate multiple humanoid robots
- **Learning from Demonstration**: Implement imitation learning capabilities
- **Adaptive Behavior**: Create behaviors that adapt based on environment or user feedback
- **Human-Robot Collaboration**: Implement collaborative tasks between humans and robots
- **Long-term Autonomy**: Design the system for sustained operation with self-monitoring

## Summary

These capstone exercises integrate all modules of the Physical AI & Humanoid Robotics textbook:

- Voice command processing and cognitive planning
- Multi-modal perception and sensor fusion
- Humanoid-specific behaviors and control
- System integration and validation

Completing these exercises will demonstrate your ability to create a complete autonomous humanoid system that can understand natural language commands, perceive its environment, plan appropriate actions, and execute them safely. This represents the culmination of all the concepts learned throughout the textbook.
