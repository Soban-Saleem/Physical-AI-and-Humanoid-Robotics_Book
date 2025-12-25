# Module 2 Exercises: The Digital Twin (Gazebo & Unity)

This section provides hands-on exercises to reinforce the concepts learned in Module 2. These exercises will help you practice and apply the simulation concepts covered in the previous sections.

## Exercise 1: Create a Basic Gazebo World

### Objective
Create a simple Gazebo world with basic physics properties and a few objects.

### Requirements
- Create a world file that includes physics properties
- Add at least 3 different objects (e.g., box, sphere, cylinder)
- Configure appropriate mass and inertial properties for each object
- Include proper lighting and visual elements
- Test the world in Gazebo to ensure objects behave correctly

### Steps to Complete
- Create a new directory for your world: `~/gazebo_worlds/basic_world`
- Create a world file named `basic_environment.sdf`
- Define physics properties for the world (gravity, time step, etc.)
- Add a ground plane and sky
- Add at least 3 objects with different shapes and properties
- Launch Gazebo with your world to verify it works correctly

### Solution Template

Create the world file:

```xml
<?xml version="1.0" ?>
<sdf version="1.7">
  <world name="basic_environment">
    <!-- Physics engine configuration -->
    <physics type="ode">
      <gravity>0 0 -9.8</gravity>
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>
    </physics>

    <!-- Include standard models -->
    <include>
      <uri>model://ground_plane</uri>
    </include>
    
    <include>
      <uri>model://sun</uri>
    </include>

    <!-- Add a static box -->
    <model name="static_box">
      <static>true</static>
      <pose>2 0 0.5 0 0 0</pose>
      <link name="link">
        <inertial>
          <mass>1.0</mass>
          <inertia>
            <ixx>0.0833333</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>0.0833333</iyy>
            <iyz>0</iyz>
            <izz>0.0833333</izz>
          </inertia>
        </inertial>
        <collision name="collision">
          <geometry>
            <box>
              <size>1 1 1</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>1 1 1</size>
            </box>
          </geometry>
          <material>
            <ambient>1 0 0 1</ambient>
            <diffuse>1 0 0 1</diffuse>
          </material>
        </visual>
      </link>
    </model>

    <!-- Add a dynamic sphere -->
    <model name="dynamic_sphere">
      <pose>-2 0 2 0 0 0</pose>
      <link name="link">
        <inertial>
          <mass>0.5</mass>
          <inertia>
            <ixx>0.1</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>0.1</iyy>
            <iyz>0</iyz>
            <izz>0.1</izz>
          </inertia>
        </inertial>
        <collision name="collision">
          <geometry>
            <sphere>
              <radius>0.25</radius>
            </sphere>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <sphere>
              <radius>0.25</radius>
            </sphere>
          </geometry>
          <material>
            <ambient>0 1 0 1</ambient>
            <diffuse>0 1 0 1</diffuse>
          </material>
        </visual>
      </link>
    </model>

    <!-- Add a dynamic cylinder -->
    <model name="dynamic_cylinder">
      <pose>0 2 1 0 0 0</pose>
      <link name="link">
        <inertial>
          <mass>0.8</mass>
          <inertia>
            <ixx>0.045</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>0.045</iyy>
            <iyz>0</iyz>
            <izz>0.02</izz>
          </inertia>
        </inertial>
        <collision name="collision">
          <geometry>
            <cylinder>
              <radius>0.2</radius>
              <length>0.5</length>
            </cylinder>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <cylinder>
              <radius>0.2</radius>
              <length>0.5</length>
            </cylinder>
          </geometry>
          <material>
            <ambient>0 0 1 1</ambient>
            <diffuse>0 0 1 1</diffuse>
          </material>
        </visual>
      </link>
    </model>
  </world>
</sdf>
```

### Expected Output
When you launch Gazebo with this world, you should see:
- A static red box that doesn't move
- A dynamic green sphere that falls due to gravity
- A dynamic blue cylinder that also falls due to gravity
- Proper lighting and ground plane

## Exercise 2: Implement Sensor Simulation

### Objective
Add various sensors to a robot model and verify that they publish data correctly.

### Requirements
- Create a simple robot model with at least 3 different sensor types
- Implement a LiDAR sensor (ray/camera type)
- Implement a camera sensor
- Implement an IMU sensor
- Verify that sensors are publishing data to appropriate topics

### Steps to Complete
1. Create a robot model file with basic structure
2. Add a LiDAR sensor configuration
3. Add a camera sensor configuration
4. Add an IMU sensor configuration
5. Test the sensors by launching Gazebo and checking topic output

### Solution Template

```xml
<?xml version="1.0"?>
<robot name="sensor_test_robot">
  <!-- Base link -->
  <link name="base_link">
    <inertial>
      <mass value="10.0"/>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <inertia ixx="0.4" ixy="0" ixz="0" iyy="0.4" iyz="0" izz="0.4"/>
    </inertial>
    
    <visual>
      <origin xyz="0 0 0.1" rpy="0 0 0"/>
      <geometry>
        <box size="0.5 0.5 0.2"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 1 1"/>
      </material>
    </visual>
    
    <collision>
      <origin xyz="0 0 0.1" rpy="0 0 0"/>
      <geometry>
        <box size="0.5 0.5 0.2"/>
      </geometry>
    </collision>
  </link>

  <!-- LiDAR sensor -->
  <sensor name="lidar" type="ray">
    <pose>0.2 0 0.2 0 0 0</pose>
    <ray>
      <scan>
        <horizontal>
          <samples>360</samples>
          <resolution>1</resolution>
          <min_angle>-3.14159</min_angle>
          <max_angle>3.14159</max_angle>
        </horizontal>
      </scan>
      <range>
        <min>0.1</min>
        <max>10.0</max>
        <resolution>0.01</resolution>
      </range>
    </ray>
    <always_on>true</always_on>
    <visualize>true</visualize>
    <update_rate>10</update_rate>
  </sensor>

  <!-- Camera sensor -->
  <sensor name="camera" type="camera">
    <pose>0.2 0 0.3 0 0 0</pose>
    <camera name="head_camera">
      <horizontal_fov>1.047</horizontal_fov>  <!-- 60 degrees -->
      <image>
        <width>640</width>
        <height>480</height>
        <format>R8G8B8</format>
      </image>
      <clip>
        <near>0.1</near>
        <far>100</far>
      </clip>
    </camera>
    <always_on>true</always_on>
    <update_rate>30</update_rate>
    <visualize>true</visualize>
  </sensor>

  <!-- IMU sensor -->
  <sensor name="imu_sensor" type="imu">
    <pose>0 0 0.1 0 0 0</pose>
    <always_on>true</always_on>
    <update_rate>100</update_rate>
    <imu>
      <angular_velocity>
        <x>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>2e-4</stddev>
            <bias_mean>0.0000075</bias_mean>
            <bias_stddev>0.0000008</bias_stddev>
          </noise>
        </x>
        <y>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>2e-4</stddev>
            <bias_mean>0.0000075</bias_mean>
            <bias_stddev>0.0000008</bias_stddev>
          </noise>
        </y>
        <z>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>2e-4</stddev>
            <bias_mean>0.0000075</bias_mean>
            <bias_stddev>0.0000008</bias_stddev>
          </noise>
        </z>
      </angular_velocity>
      <linear_acceleration>
        <x>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>1.7e-2</stddev>
            <bias_mean>0.1</bias_mean>
            <bias_stddev>0.001</bias_stddev>
          </noise>
        </x>
        <y>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>1.7e-2</stddev>
            <bias_mean>0.1</bias_mean>
            <bias_stddev>0.001</bias_stddev>
          </noise>
        </y>
        <z>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>1.7e-2</stddev>
            <bias_mean>0.1</bias_mean>
            <bias_stddev>0.001</bias_stddev>
          </noise>
        </z>
      </linear_acceleration>
    </imu>
  </sensor>
</robot>
```

### Verification Steps
1. Launch Gazebo with the robot model
2. Check available topics: `gz topic -l`
3. Verify sensor topics are publishing: 
   - LiDAR: `/lidar/scan` or similar
   - Camera: `/camera/head_camera/image` or similar
   - IMU: `/imu/data` or similar
4. Echo one of the topics to verify data: `gz topic -e -t /lidar/scan`

## Exercise 3: Connect Robot to ROS 2

### Objective
Connect the simulated robot to ROS 2 nodes using the Gazebo Bridge.

### Requirements
- Set up the Gazebo Bridge to connect sensor topics to ROS 2
- Create a ROS 2 node that subscribes to sensor data
- Create a ROS 2 node that publishes commands to the robot
- Verify bidirectional communication between Gazebo and ROS 2

### Steps to Complete
1. Create a bridge configuration file
2. Launch the bridge
3. Create a ROS 2 node that subscribes to sensor data
4. Create a ROS 2 node that publishes commands to the robot
5. Test communication between ROS 2 and Gazebo

### Solution Template

Bridge configuration file (`sensor_bridge_config.yaml`):
```yaml
- ros_topic_name: "/cmd_vel"
  gz_topic_name: "/model/robot/cmd_vel"
  ros_type_name: "geometry_msgs/msg/Twist"
  gz_type_name: "gz.msgs.Twist"
  direction: ROS_TO_GZ

- ros_topic_name: "/scan"
  gz_topic_name: "/lidar/scan"
  ros_type_name: "sensor_msgs/msg/LaserScan"
  gz_type_name: "gz.msgs.LaserScan"
  direction: GZ_TO_ROS

- ros_topic_name: "/camera/image_raw"
  gz_topic_name: "/camera/head_camera/image"
  ros_type_name: "sensor_msgs/msg/Image"
  gz_type_name: "gz.msgs.Image"
  direction: GZ_TO_ROS
```

ROS 2 subscriber node:
```python
# sensor_subscriber.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Image
from cv_bridge import CvBridge
import numpy as np

class SensorSubscriber(Node):
    def __init__(self):
        super().__init__('sensor_subscriber')
        
        # Create subscribers
        self.scan_subscription = self.create_subscription(
            LaserScan, 
            '/scan', 
            self.scan_callback, 
            10)
        
        self.image_subscription = self.create_subscription(
            Image, 
            '/camera/image_raw', 
            self.image_callback, 
            10)
        
        self.bridge = CvBridge()
        self.get_logger().info('Sensor subscriber initialized')

    def scan_callback(self, msg):
        # Process LiDAR data
        self.get_logger().info(f'Received scan with {len(msg.ranges)} ranges')
        
        # Example: find minimum distance
        if msg.ranges:
            min_distance = min([r for r in msg.ranges if r > 0])
            self.get_logger().info(f'Minimum distance: {min_distance:.2f}m')

    def image_callback(self, msg):
        # Process camera data
        self.get_logger().info(f'Received image: {msg.width}x{msg.height}')
        
        # Convert to OpenCV format for processing
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            # Further image processing would go here
        except Exception as e:
            self.get_logger().error(f'Error converting image: {e}')

def main(args=None):
    rclpy.init(args=args)
    sensor_subscriber = SensorSubscriber()
    
    try:
        rclpy.spin(sensor_subscriber)
    except KeyboardInterrupt:
        sensor_subscriber.get_logger().info('Shutting down sensor subscriber...')
    finally:
        sensor_subscriber.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Exercise 4: Implement Physics-Based Robot Control

### Objective
Create a simulation that demonstrates realistic physics-based robot control.

### Requirements
- Implement a differential drive robot with realistic physics
- Configure appropriate mass, friction, and inertial properties
- Add wheel joints with proper transmission
- Connect to ROS 2 for control using the diff_drive_controller
- Test that the robot moves realistically in simulation

### Steps to Complete
1. Create a robot model with differential drive configuration
2. Configure realistic physical properties for the robot
3. Add transmissions for the wheels
4. Create a launch file that starts Gazebo with the robot
5. Test robot control via ROS 2 topics

### Solution Template

Differential drive robot model:
```xml
<?xml version="1.0"?>
<robot name="diff_drive_robot" xmlns:xacro="http://www.ros.org/wiki/xacro">
  <!-- Base link -->
  <link name="base_link">
    <inertial>
      <mass value="10.0"/>
      <origin xyz="0 0 0.1" rpy="0 0 0"/>
      <inertia ixx="0.4" ixy="0" ixz="0" iyy="0.4" iyz="0" izz="0.4"/>
    </inertial>
    
    <visual>
      <origin xyz="0 0 0.1" rpy="0 0 0"/>
      <geometry>
        <box size="0.5 0.4 0.2"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 1 1"/>
      </material>
    </visual>
    
    <collision>
      <origin xyz="0 0 0.1" rpy="0 0 0"/>
      <geometry>
        <box size="0.5 0.4 0.2"/>
      </geometry>
    </collision>
  </link>

  <!-- Left wheel -->
  <link name="left_wheel">
    <inertial>
      <mass value="1.0"/>
      <origin xyz="0 0 0" rpy="0 1.5708 1.5708"/>
      <inertia ixx="0.1" ixy="0" ixz="0" iyy="0.1" iyz="0" izz="0.1"/>
    </inertial>
    
    <visual>
      <origin xyz="0 0 0" rpy="0 1.5708 1.5708"/>
      <geometry>
        <cylinder radius="0.1" length="0.05"/>
      </geometry>
    </visual>
    
    <collision>
      <origin xyz="0 0 0" rpy="0 1.5708 1.5708"/>
      <geometry>
        <cylinder radius="0.1" length="0.05"/>
      </geometry>
    </collision>
  </link>

  <!-- Right wheel -->
  <link name="right_wheel">
    <inertial>
      <mass value="1.0"/>
      <origin xyz="0 0 0" rpy="0 1.5708 1.5708"/>
      <inertia ixx="0.1" ixy="0" ixz="0" iyy="0.1" iyz="0" izz="0.1"/>
    </inertial>
    
    <visual>
      <origin xyz="0 0 0" rpy="0 1.5708 1.5708"/>
      <geometry>
        <cylinder radius="0.1" length="0.05"/>
      </geometry>
    </visual>
    
    <collision>
      <origin xyz="0 0 0" rpy="0 1.5708 1.5708"/>
      <geometry>
        <cylinder radius="0.1" length="0.05"/>
      </geometry>
    </collision>
  </link>

  <!-- Wheel joints -->
  <joint name="left_wheel_joint" type="continuous">
    <parent link="base_link"/>
    <child link="left_wheel"/>
    <origin xyz="0 0.25 0" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
  </joint>

  <joint name="right_wheel_joint" type="continuous">
    <parent link="base_link"/>
    <child link="right_wheel"/>
    <origin xyz="0 -0.25 0" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
  </joint>

  <!-- Gazebo plugins -->
  <gazebo>
    <plugin name="diff_drive" filename="libgazebo_ros_diff_drive.so">
      <commandTopic>cmd_vel</commandTopic>
      <odometryTopic>odom</odometryTopic>
      <odometryFrame>odom</odometryFrame>
      <robotBaseFrame>base_link</robotBaseFrame>
      <publishOdomTF>true</publishOdomTF>
      <publishWheelTF>true</publishWheelTF>
      <publishWheelJointState>true</publishWheelJointState>
      <legacyMode>false</legacyMode>
      <updateRate>30</updateRate>
      <leftJoint>left_wheel_joint</leftJoint>
      <rightJoint>right_wheel_joint</rightJoint>
      <wheelSeparation>0.5</wheelSeparation>
      <wheelDiameter>0.2</wheelDiameter>
      <wheelAcceleration>1.0</wheelAcceleration>
      <wheelTorque>5.0</wheelTorque>
    </plugin>
  </gazebo>
</robot>
```

## Exercise 5: Integration Challenge

### Objective
Combine all concepts from Module 2 to create a complete simulation scenario with a robot that navigates through an environment using sensor data.

### Requirements
- Create a world with obstacles
- Add a robot with multiple sensors (LiDAR, camera, IMU)
- Implement a ROS 2 node that processes sensor data to navigate
- Connect everything using the Gazebo Bridge
- Demonstrate the robot successfully navigating around obstacles

### Solution Approach
1. Create a world file with obstacles
2. Create a robot with the sensors and differential drive
3. Set up the Gazebo Bridge for all necessary topics
4. Implement a navigation node that uses sensor data to avoid obstacles
5. Test the complete system

### Expected Output
A robot that uses its sensors to detect obstacles and navigate around them in the simulated environment.

## Self-Assessment Questions

1. How do you configure the physics properties in a Gazebo world file?
2. What are the key differences between static and dynamic objects in Gazebo?
3. How do you connect Gazebo sensors to ROS 2 topics?
4. What are the important considerations when setting up the Gazebo Bridge?
5. How do you verify that sensor simulation is working correctly?

## Advanced Challenges (Optional)

1. **Advanced Sensors**: Add a depth camera or 3D LiDAR to your robot
2. **Realistic Materials**: Configure realistic materials and lighting for better visual simulation
3. **Terrain Simulation**: Create a world with uneven terrain and test robot traversal
4. **Multi-Robot Simulation**: Simulate multiple robots in the same environment with coordination
5. **Hardware-in-the-Loop**: Design the simulation to allow easy transition to real hardware testing

## Summary

These exercises provide hands-on practice with Gazebo simulation concepts:
- Creating world files with physics properties
- Implementing various sensor types
- Connecting simulation to ROS 2
- Configuring realistic physics properties
- Validating sensor and control systems

Completing these exercises will strengthen your understanding of simulation for robotics and prepare you for the more advanced topics in subsequent modules.