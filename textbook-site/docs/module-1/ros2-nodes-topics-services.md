# ROS 2 Nodes, Topics, and Services

In this section, we'll explore the core communication patterns in ROS 2: nodes, topics, and services. These concepts form the foundation of how components in a robotic system communicate with each other, which is essential for creating integrated humanoid robotics systems.

## Learning Objectives

By the end of this section, you will be able to:
- Create and run ROS 2 nodes for different robot components
- Implement communication between nodes using topics and services
- Design appropriate communication patterns for different use cases
- Understand the differences between topics and services and when to use each
- Apply ROS 2 best practices for effective communication
- Create custom message types for humanoid robotics applications

## Nodes in ROS 2

A node is a process that performs computation. Nodes are the fundamental building blocks of ROS programs. In a humanoid robot system, you might have separate nodes for:

- Perception (computer vision, sensor processing)
- Planning (motion planning, path planning)
- Control (motor control, actuator commands)
- Communication (networking, data logging)

### Creating Nodes

Let's look at how to create nodes in more detail:

```python
# File: humanoid_ws/src/humanoid_basics/humanoid_basics/joint_controller.py

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration


class JointController(Node):

    def __init__(self):
        super().__init__('joint_controller')
        
        # Subscribe to joint commands
        self.subscription = self.create_subscription(
            JointTrajectory,
            'joint_trajectory_commands',
            self.joint_command_callback,
            10)
        
        # Publish joint states
        self.publisher = self.create_publisher(
            JointState,
            'joint_states',
            10)
        
        # Timer for publishing joint states
        self.timer = self.create_timer(0.1, self.publish_joint_states)
        
        # Initialize joint positions
        self.joint_positions = [0.0] * 12  # For a 12-DOF humanoid
        self.joint_names = [
            'left_hip_roll', 'left_hip_yaw', 'left_hip_pitch',
            'left_knee', 'left_ankle_pitch', 'left_ankle_roll',
            'right_hip_roll', 'right_hip_yaw', 'right_hip_pitch',
            'right_knee', 'right_ankle_pitch', 'right_ankle_roll'
        ]
        
        self.get_logger().info('Joint Controller initialized')

    def joint_command_callback(self, msg):
        """Handle incoming joint trajectory commands"""
        self.get_logger().info(f'Received trajectory with {len(msg.points)} points')
        
        # In a real implementation, this would command the physical joints
        # For simulation, we'll just update our internal state
        if len(msg.points) > 0:
            # Take the first point as the target position
            target_point = msg.points[0]
            if len(target_point.positions) == len(self.joint_positions):
                self.joint_positions = list(target_point.positions)
                self.get_logger().info('Updated joint positions')

    def publish_joint_states(self):
        """Publish current joint states"""
        msg = JointState()
        msg.name = self.joint_names
        msg.position = self.joint_positions
        msg.velocity = [0.0] * len(self.joint_positions)  # Simplified
        msg.effort = [0.0] * len(self.joint_positions)   # Simplified
        
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        
        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    
    joint_controller = JointController()
    
    try:
        rclpy.spin(joint_controller)
    except KeyboardInterrupt:
        joint_controller.get_logger().info('Shutting down joint controller...')
    finally:
        joint_controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## Topics: Asynchronous Communication

Topics enable asynchronous, many-to-many communication between nodes. Publishers send messages to a topic, and subscribers receive messages from a topic. This pattern is ideal for streaming data like sensor readings or robot joint positions.

### Topic Communication Characteristics

- **Many-to-many**: Multiple publishers can publish to a topic, and multiple subscribers can subscribe to the same topic
- **Asynchronous**: Publishers and subscribers don't need to be synchronized
- **Streaming**: Ideal for continuous data streams like sensor data, joint states, or camera feeds

### Quality of Service (QoS) for Topics

Different types of data require different QoS settings:

```python
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

# For sensor data (high frequency, may drop some messages)
sensor_qos = QoSProfile(
    depth=10,
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    history=QoSHistoryPolicy.KEEP_LAST
)

# For critical commands (must be reliable)
command_qos = QoSProfile(
    depth=10,
    reliability=QoSReliabilityPolicy.RELIABLE,
    history=QoSHistoryPolicy.KEEP_LAST
)

# For historical data (keep all messages)
historical_qos = QoSProfile(
    depth=1000,
    reliability=QoSReliabilityPolicy.RELIABLE,
    history=QoSHistoryPolicy.KEEP_ALL
)
```

### Creating Custom Message Types

For humanoid robotics applications, you might need custom message types. Let's create one for humanoid-specific data:

```python
# Create a new file: humanoid_msgs/msg/HumanoidState.msg
# This would go in a separate package

# In humanoid_msgs/msg/HumanoidState.msg
float64[] joint_positions
float64[] joint_velocities
float64[] joint_efforts
geometry_msgs/Pose com_pose  # Center of mass pose
geometry_msgs/Twist com_twist  # Center of mass velocity
bool in_contact  # Whether feet are in contact with ground
float64 balance_score  # Balance stability measure
```

To use this custom message, you would create a separate package for messages:

```bash
cd ~/humanoid_ws/src
ros2 pkg create --build-type ament_cmake humanoid_msgs
```

Then add the message file to `humanoid_msgs/msg/HumanoidState.msg` and update the `CMakeLists.txt` and `package.xml` accordingly.

## Services: Synchronous Request-Response

Services provide synchronous, request-response communication. A client sends a request to a service, and the service returns a response. This pattern is ideal for actions that have a clear start and end, such as requesting a robot to move to a specific location.

### Creating a Service

Let's create a service for commanding the humanoid to move to a specific pose:

First, create the service definition:

```
# In humanoid_msgs/srv/HumanoidMoveTo.srv
geometry_msgs/Pose target_pose
float64 max_duration
---
bool success
string message
```

Then implement the service server:

```python
# File: humanoid_ws/src/humanoid_basics/humanoid_basics/move_to_service.py

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from humanoid_msgs.srv import HumanoidMoveTo


class MoveToService(Node):

    def __init__(self):
        super().__init__('move_to_service')
        
        # Create service
        self.srv = self.create_service(
            HumanoidMoveTo, 
            'move_to_pose', 
            self.move_to_pose_callback)
        
        # Store current pose
        self.current_pose = Pose()
        self.get_logger().info('Move To Service initialized')

    def move_to_pose_callback(self, request, response):
        """Handle move to pose requests"""
        target_pose = request.target_pose
        self.get_logger().info(
            f'Received move request to position: '
            f'({target_pose.position.x}, {target_pose.position.y}, {target_pose.position.z})'
        )
        
        # In a real implementation, this would plan and execute the movement
        # For now, we'll just simulate success
        try:
            # Simulate movement execution
            self.execute_movement(target_pose)
            
            # Update current pose
            self.current_pose = target_pose
            
            response.success = True
            response.message = 'Successfully moved to pose'
        except Exception as e:
            response.success = False
            response.message = f'Movement failed: {str(e)}'
        
        return response

    def execute_movement(self, target_pose):
        """Execute the movement to target pose (simulated)"""
        # This would contain the actual movement planning and execution logic
        # For now, just simulate with a sleep
        import time
        time.sleep(0.1)  # Simulate computation time
        

def main(args=None):
    rclpy.init(args=args)
    
    move_to_service = MoveToService()
    
    try:
        rclpy.spin(move_to_service)
    except KeyboardInterrupt:
        move_to_service.get_logger().info('Shutting down move to service...')
    finally:
        move_to_service.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### Creating a Service Client

Now let's create a client that calls this service:

```python
# File: humanoid_ws/src/humanoid_basics/humanoid_basics/move_to_client.py

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from humanoid_msgs.srv import HumanoidMoveTo


class MoveToClient(Node):

    def __init__(self):
        super().__init__('move_to_client')
        self.cli = self.create_client(HumanoidMoveTo, 'move_to_pose')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')
        
        self.req = HumanoidMoveTo.Request()

    def send_request(self, x, y, z, ox=0.0, oy=0.0, oz=0.0, ow=1.0):
        self.req.target_pose.position.x = x
        self.req.target_pose.position.y = y
        self.req.target_pose.position.z = z
        self.req.target_pose.orientation.x = ox
        self.req.target_pose.orientation.y = oy
        self.req.target_pose.orientation.z = oz
        self.req.target_pose.orientation.w = ow
        self.req.max_duration = 10.0  # seconds
        
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        
        return self.future.result()


def main():
    rclpy.init()

    move_to_client = MoveToClient()
    
    # Request to move to a specific pose
    response = move_to_client.send_request(1.0, 0.0, 0.0)
    
    if response:
        if response.success:
            print(f'Success: {response.message}')
        else:
            print(f'Failure: {response.message}')
    else:
        print('Service call failed')

    move_to_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## Communication Pattern Selection

Choosing the right communication pattern is crucial for effective system design:

### When to Use Topics
- Streaming data (sensor readings, joint states, camera feeds)
- Data that multiple nodes need simultaneously
- Situations where some data loss is acceptable
- Real-time applications where latency is critical

### When to Use Services
- Request-response patterns with clear start/end
- Actions that modify system state
- Situations where reliability is critical
- Configuration changes or queries

### When to Use Actions (covered in Module 3)
- Long-running tasks that provide feedback
- Tasks that can be canceled or preempted
- Operations with intermediate progress reporting

## Best Practices for Communication

### 1. Message Design
- Keep messages small to reduce network overhead
- Use appropriate data types (e.g., float32 vs float64)
- Design messages for extensibility (add new fields rather than changing existing ones)

### 2. Topic Naming
- Use descriptive, hierarchical names: `/robot1/left_arm/joint_states`
- Follow consistent naming conventions
- Use underscores, not hyphens or camelCase

### 3. Error Handling
- Implement proper error handling in all nodes
- Use appropriate logging levels
- Handle connection failures gracefully

### 4. Resource Management
- Clean up resources properly in node destruction
- Use appropriate queue sizes for publishers/subscribers
- Monitor resource usage in complex systems

## Example: Humanoid Perception Node

Let's create a more complex example that combines topics and services:

```python
# File: humanoid_ws/src/humanoid_basics/humanoid_basics/perception_node.py

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2, LaserScan
from geometry_msgs.msg import PointStamped
from humanoid_msgs.srv import ObjectDetection
from humanoid_msgs.msg import HumanoidState
import numpy as np
from cv_bridge import CvBridge


class PerceptionNode(Node):

    def __init__(self):
        super().__init__('perception_node')
        
        # Initialize CV bridge
        self.bridge = CvBridge()
        
        # Publishers
        self.object_position_pub = self.create_publisher(PointStamped, 'detected_object_position', 10)
        self.humanoid_state_sub = self.create_subscription(
            HumanoidState, 'humanoid_state', self.state_callback, 10)
        
        # Subscribers
        self.image_sub = self.create_subscription(Image, 'camera/image_raw', self.image_callback, 10)
        self.pointcloud_sub = self.create_subscription(
            PointCloud2, 'lidar/points', self.pointcloud_callback, 10)
        
        # Service
        self.detection_srv = self.create_service(
            ObjectDetection, 'detect_object', self.detect_object_callback)
        
        # Internal state
        self.current_state = None
        self.latest_image = None
        self.latest_pointcloud = None
        self.detected_objects = []
        
        self.get_logger().info('Perception Node initialized')

    def state_callback(self, msg):
        """Handle humanoid state updates"""
        self.current_state = msg

    def image_callback(self, msg):
        """Process incoming camera images"""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            # In a real implementation, this would run object detection
            # For now, we'll just store the image
            self.latest_image = cv_image
        except Exception as e:
            self.get_logger().error(f'Error processing image: {e}')

    def pointcloud_callback(self, msg):
        """Process incoming point cloud data"""
        # In a real implementation, this would process the point cloud
        # For now, we'll just store the message
        self.latest_pointcloud = msg

    def detect_object_callback(self, request, response):
        """Detect objects in the environment"""
        object_name = request.object_name
        
        # In a real implementation, this would perform object detection
        # For now, we'll simulate detection
        if self.current_state and self.latest_image is not None:
            # Simulate detection at current head position
            detected_pos = PointStamped()
            detected_pos.header.stamp = self.get_clock().now().to_msg()
            detected_pos.header.frame_id = 'head_link'
            detected_pos.point.x = 1.0  # 1 meter in front of head
            detected_pos.point.y = 0.0
            detected_pos.point.z = 0.0
            
            # Publish the detected position
            self.object_position_pub.publish(detected_pos)
            
            response.success = True
            response.message = f'Detected {object_name} at position'
            response.position = detected_pos.point
        else:
            response.success = False
            response.message = 'No image or state data available'
            response.position = Point()
        
        return response


def main(args=None):
    rclpy.init(args=args)
    
    perception_node = PerceptionNode()
    
    try:
        rclpy.spin(perception_node)
    except KeyboardInterrupt:
        perception_node.get_logger().info('Shutting down perception node...')
    finally:
        perception_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## Summary

In this section, we've explored the core communication patterns in ROS 2: nodes, topics, and services. We've seen how to create nodes that communicate with each other using these patterns, and we've discussed when to use each pattern based on the requirements of your application.

Topics are ideal for streaming data and asynchronous communication, while services are better for request-response patterns with clear start and end states. Understanding these patterns is crucial for designing effective robotic systems, especially complex ones like humanoid robots that require coordination between many different components.

In the next section, we'll explore how to integrate Python with ROS 2 using the rclpy library, which is essential for creating the AI components of our humanoid robotics system.