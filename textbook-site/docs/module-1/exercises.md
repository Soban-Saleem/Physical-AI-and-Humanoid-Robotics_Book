# Module 1 Exercises: The Robotic Nervous System (ROS 2)

This section provides hands-on exercises to reinforce the concepts learned in Module 1. These exercises are designed to help you practice and apply the ROS 2 concepts covered in the previous sections.

## Exercise 1: Basic ROS 2 Node Creation

### Objective
Create a simple ROS 2 publisher node that publishes messages to a topic and a subscriber node that receives and processes these messages.

### Requirements
- Create a new ROS 2 package called `humanoid_basics`
- Create a publisher node that publishes counter values to `/counter` topic
- Create a subscriber node that subscribes to `/counter` topic and prints received values
- Use appropriate message types from `std_msgs`
- Include proper node initialization and cleanup

### Steps to Complete
1. Create a new package: `ros2 pkg create --build-type ament_python humanoid_basics`
2. Create the publisher node file: `humanoid_basics/humanoid_basics/counter_publisher.py`
3. Create the subscriber node file: `humanoid_basics/humanoid_basics/counter_subscriber.py`
4. Update the `setup.py` file to make the nodes executable
5. Build and run the nodes to verify they work correctly

### Expected Output
The subscriber should print the counter values published by the publisher node, incrementing approximately once per second.

### Solution Template

Publisher node code:
```python
# File: humanoid_basics/humanoid_basics/counter_publisher.py

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class CounterPublisher(Node):

    def __init__(self):
        super().__init__('counter_publisher')
        self.publisher_ = self.create_publisher(String, 'counter', 10)
        timer_period = 1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.counter = 0

    def timer_callback(self):
        msg = String()
        msg.data = f'Counter value: {self.counter}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.counter += 1


def main(args=None):
    rclpy.init(args=args)
    
    counter_publisher = CounterPublisher()
    
    try:
        rclpy.spin(counter_publisher)
    except KeyboardInterrupt:
        counter_publisher.get_logger().info('Shutting down counter publisher...')
    finally:
        counter_publisher.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

Subscriber node code:
```python
# File: humanoid_basics/humanoid_basics/counter_subscriber.py

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class CounterSubscriber(Node):

    def __init__(self):
        super().__init__('counter_subscriber')
        self.subscription = self.create_subscription(
            String,
            'counter',
            self.listener_callback,
            10)
        self.subscription  # Prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info(f'Received: "{msg.data}"')


def main(args=None):
    rclpy.init(args=args)
    
    counter_subscriber = CounterSubscriber()
    
    try:
        rclpy.spin(counter_subscriber)
    except KeyboardInterrupt:
        counter_subscriber.get_logger().info('Shutting down counter subscriber...')
    finally:
        counter_subscriber.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## Exercise 2: Robot State Publisher

### Objective
Create a node that publishes robot joint states and implements the robot state publisher pattern.

### Requirements
- Create a publisher that publishes `sensor_msgs/JointState` messages
- Simulate joint positions for a simple robot (e.g., 6-DOF arm)
- Include proper header information with timestamps
- Implement a timer to publish joint states at a consistent rate

### Solution Steps
1. Create a new node file: `humanoid_basics/humanoid_basics/robot_state_publisher.py`
2. Import the `sensor_msgs.msg.JointState` message type
3. Implement a timer callback that publishes joint state messages
4. Simulate changing joint positions over time
5. Test the node and verify joint states are published correctly

### Expected Output
The node should publish joint state messages at a consistent rate (e.g., 50Hz) with changing joint positions that simulate a moving robot.

## Exercise 3: Service Implementation

### Objective
Implement a ROS 2 service that calculates the distance between two points and returns the result.

### Requirements
- Define a custom service message for distance calculation
- Implement a service server that calculates Euclidean distance
- Create a service client that calls the service with two points
- Handle potential errors in the service implementation

### Steps to Complete
1. Create a custom service definition file: `humanoid_msgs/srv/CalculateDistance.srv`
2. Implement the service server node
3. Implement the service client node
4. Test the service by calling it with different points

### Service Definition Example
```
# CalculateDistance.srv
geometry_msgs/Point point1
geometry_msgs/Point point2
---
float64 distance
bool success
string message
```

## Exercise 4: URDF Robot Model

### Objective
Create a URDF file for a simple robot and visualize it in RViz2.

### Requirements
- Create a URDF file for a simple 2-wheeled robot
- Include visual and collision properties for each link
- Define appropriate joints between links
- Use proper inertial properties
- Validate the URDF file using `check_urdf`

### Steps to Complete
1. Create a URDF file: `humanoid_basics/urdf/simple_robot.urdf`
2. Define a base link with visual, collision, and inertial properties
3. Define two wheel links and connect them with joints
4. Validate the URDF file
5. Launch RViz2 to visualize the robot

### Example URDF Structure
```xml
<?xml version="1.0"?>
<robot name="simple_robot">
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.5 0.3 0.15"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 1 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.5 0.3 0.15"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.1" ixy="0.0" ixz="0.0" iyy="0.1" iyz="0.0" izz="0.1"/>
    </inertial>
  </link>
  
  <!-- Add wheels and joints -->
  <link name="wheel_left">
    <visual>
      <geometry>
        <cylinder radius="0.1" length="0.05"/>
      </geometry>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.1" length="0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.2"/>
      <inertia ixx="0.001" ixy="0.0" ixz="0.0" iyy="0.001" iyz="0.0" izz="0.001"/>
    </inertial>
  </link>
  
  <joint name="left_wheel_joint" type="continuous">
    <parent link="base_link"/>
    <child link="wheel_left"/>
    <origin xyz="0 -0.2 -0.05" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
  </joint>
  
  <!-- Add right wheel -->
  <link name="wheel_right">
    <visual>
      <geometry>
        <cylinder radius="0.1" length="0.05"/>
      </geometry>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.1" length="0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.2"/>
      <inertia ixx="0.001" ixy="0.0" ixz="0.0" iyy="0.001" iyz="0.0" izz="0.001"/>
    </inertial>
  </link>
  
  <joint name="right_wheel_joint" type="continuous">
    <parent link="base_link"/>
    <child link="wheel_right"/>
    <origin xyz="0 0.2 -0.05" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
  </joint>
</robot>
```

## Exercise 5: Python Integration with AI Libraries

### Objective
Create a ROS 2 node that processes sensor data using a basic AI algorithm (e.g., clustering or classification).

### Requirements
- Create a node that subscribes to sensor data (e.g., LaserScan)
- Use a Python AI library (like scikit-learn) to process the data
- Publish the processed results to a new topic
- Include error handling for the AI processing

### Solution Steps
1. Create a new node file: `humanoid_basics/humanoid_basics/ai_sensor_processor.py`
2. Subscribe to a sensor topic (e.g., `/scan` for LaserScan messages)
3. Implement a simple AI algorithm (e.g., K-means clustering for object detection)
4. Publish the results to a new topic
5. Test the node with simulated sensor data

### Example Implementation
```python
# File: humanoid_basics/humanoid_basics/ai_sensor_processor.py

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
import numpy as np
from sklearn.cluster import DBSCAN


class AISensorProcessor(Node):

    def __init__(self):
        super().__init__('ai_sensor_processor')
        
        self.subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,
            10)
        
        self.publisher = self.create_publisher(String, 'ai_processing_result', 10)
        
        # Initialize AI model
        self.ai_model = DBSCAN(eps=0.5, min_samples=3)
        
        self.get_logger().info('AI Sensor Processor initialized')

    def scan_callback(self, msg):
        """Process laser scan data using AI algorithm"""
        try:
            # Extract valid ranges (not inf or nan)
            valid_ranges = []
            valid_angles = []
            
            for i, range_val in enumerate(msg.ranges):
                if np.isfinite(range_val):
                    angle = msg.angle_min + i * msg.angle_increment
                    valid_ranges.append(range_val)
                    valid_angles.append(angle)
            
            # Convert to Cartesian coordinates
            x_points = []
            y_points = []
            for r, angle in zip(valid_ranges, valid_angles):
                x = r * np.cos(angle)
                y = r * np.sin(angle)
                x_points.append(x)
                y_points.append(y)
            
            # Combine x and y into points array
            points = np.column_stack((x_points, y_points))
            
            # Apply clustering to detect objects
            if len(points) > 0:
                clusters = self.ai_model.fit_predict(points)
                num_clusters = len(set(clusters)) - (1 if -1 in clusters else 0)
                
                # Publish result
                result_msg = String()
                result_msg.data = f'Detected {num_clusters} objects'
                self.publisher.publish(result_msg)
                
                self.get_logger().info(f'AI processing result: {result_msg.data}')
            else:
                self.get_logger().warn('No valid laser data to process')
                
        except Exception as e:
            self.get_logger().error(f'Error in AI processing: {str(e)}')


def main(args=None):
    rclpy.init(args=args)
    
    ai_processor = AISensorProcessor()
    
    try:
        rclpy.spin(ai_processor)
    except KeyboardInterrupt:
        ai_processor.get_logger().info('Shutting down AI sensor processor...')
    finally:
        ai_processor.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## Exercise 6: Integration Challenge

### Objective
Combine multiple concepts from this module to create a complete robot behavior node.

### Requirements
- Create a node that subscribes to sensor data
- Uses AI processing to interpret the sensor data
- Makes decisions based on the interpretation
- Publishes commands to control the robot
- Implements proper error handling and logging

### Solution Steps
1. Plan the node architecture considering all requirements
2. Create the node with multiple subscribers and publishers
3. Implement the AI processing and decision-making logic
4. Add comprehensive error handling and logging
5. Test the complete behavior in simulation

### Expected Output
A node that demonstrates a complete behavior loop: sensing → processing → decision → action, which is fundamental to robotics applications.

## Self-Assessment Questions

- How would you modify the publisher/subscriber example to handle different message types?
- What are the differences between using topics and services in ROS 2?
- Why are proper inertial properties important in URDF files?
- How would you handle errors in a ROS 2 node to ensure system stability?
- What considerations would you make for real-time performance in a humanoid robot control node?

## Advanced Challenges (Optional)

- **Real-time Performance**: Modify one of your nodes to meet real-time performance requirements (e.g., processing at 100Hz with less than 10ms latency)
- **Dynamic Parameters**: Add dynamic parameter reconfiguration to one of your nodes
- **Multi-Node Coordination**: Create a system with multiple coordinated nodes that work together
- **Simulation Integration**: Connect your nodes to a Gazebo simulation

## Summary

These exercises provide hands-on practice with the core concepts of ROS 2 covered in Module 1:
- Node creation and communication
- Topic and service patterns
- URDF robot description
- Python integration with AI libraries
- Best practices for robust implementation

Completing these exercises will strengthen your understanding of ROS 2 fundamentals and prepare you for the more advanced topics in subsequent modules.