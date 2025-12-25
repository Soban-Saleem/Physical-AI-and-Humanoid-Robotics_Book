# Sensor Simulation in Gazebo

Sensor simulation is a critical component of creating realistic digital twins for humanoid robots. In this section, we'll explore how to simulate various types of sensors that robots use to perceive their environment, including cameras, LiDAR, IMUs, and other sensors. Properly simulated sensors allow us to develop and test perception algorithms in a safe, repeatable environment before deploying to physical hardware.

## Learning Objectives

By the end of this section, you will be able to:
- Configure realistic models for various robot sensors (LiDAR, cameras, IMUs, etc.)
- Implement sensor noise and error models to increase simulation realism
- Connect simulated sensors to ROS 2 topics for processing
- Validate sensor simulation outputs against expected behavior
- Optimize sensor simulation for performance while maintaining accuracy
- Understand the differences between simulated and real sensors

## Introduction to Sensor Simulation

Robots rely on sensors to perceive their environment and understand their state. In simulation, we need to recreate these sensing capabilities with realistic properties, including:

- **Physical accuracy**: Sensors should respond to the simulated environment as real sensors would
- **Noise models**: Simulated sensors should include realistic noise characteristics
- **Performance characteristics**: Sensors should have appropriate update rates and latencies
- **Environmental interactions**: Sensors should respond to lighting, weather, and other environmental conditions

## Types of Sensors in Robotics

### Range Sensors
- **LiDAR**: Light Detection and Ranging sensors for 2D or 3D mapping
- **Sonar**: Ultrasonic sensors for proximity detection
- **Infrared**: IR sensors for distance measurement

### Vision Sensors
- **Cameras**: RGB sensors for visual perception
- **Depth Cameras**: RGB-D sensors that provide depth information
- **Stereo Cameras**: Two cameras for depth perception through triangulation
- **Fish-eye Cameras**: Wide-angle cameras for large field of view

### Inertial Sensors
- **IMU**: Inertial Measurement Unit measuring acceleration and angular velocity
- **Accelerometer**: Measures linear acceleration
- **Gyroscope**: Measures angular velocity
- **Magnetometer**: Measures magnetic field direction (compass)

### Other Sensors
- **GPS**: Global Positioning System for outdoor localization
- **Force/Torque**: Sensors for measuring forces and torques at joints
- **Encoders**: Measure joint positions and velocities

## LiDAR Simulation

LiDAR sensors are crucial for navigation, mapping, and obstacle detection in robotics.

### Basic LiDAR Configuration

```xml
<sensor name="lidar" type="ray">
  <pose>0.2 0 0.2 0 0 0</pose>
  <ray>
    <scan>
      <horizontal>
        <samples>720</samples>
        <resolution>1</resolution>
        <min_angle>-3.14159</min_angle>  <!-- -π radians -->
        <max_angle>3.14159</max_angle>   <!-- π radians -->
      </horizontal>
    </scan>
    <range>
      <min>0.1</min>
      <max>30.0</max>
      <resolution>0.01</resolution>
    </range>
  </ray>
  <always_on>true</always_on>
  <visualize>true</visualize>
  <update_rate>10</update_rate>
</sensor>
```

### Adding Noise to LiDAR

Real LiDAR sensors have noise characteristics that should be modeled in simulation:

```xml
<sensor name="lidar_with_noise" type="ray">
  <ray>
    <scan>
      <horizontal>
        <samples>720</samples>
        <resolution>1</resolution>
        <min_angle>-3.14159</min_angle>
        <max_angle>3.14159</max_angle>
      </horizontal>
    </scan>
    <range>
      <min>0.1</min>
      <max>30.0</max>
      <resolution>0.01</resolution>
    </range>
    <noise>
      <type>gaussian</type>
      <mean>0.0</mean>
      <stddev>0.01</stddev>  <!-- 1cm standard deviation -->
    </noise>
  </ray>
  <always_on>true</always_on>
  <visualize>true</visualize>
  <update_rate>10</update_rate>
</sensor>
```

### Advanced LiDAR Configuration (3D)

For 3D LiDAR sensors like those used on humanoid robots:

```xml
<sensor name="3d_lidar" type="ray">
  <pose>0.3 0 0.3 0 0 0</pose>
  <ray>
    <scan>
      <horizontal>
        <samples>1024</samples>
        <resolution>1</resolution>
        <min_angle>-3.14159</min_angle>
        <max_angle>3.14159</max_angle>
      </horizontal>
      <vertical>
        <samples>64</samples>
        <resolution>1</resolution>
        <min_angle>-0.5236</min_angle>  <!-- -30 degrees -->
        <max_angle>0.2618</max_angle>   <!-- 15 degrees -->
      </vertical>
    </scan>
    <range>
      <min>0.1</min>
      <max>100.0</max>
      <resolution>0.001</resolution>
    </range>
    <noise>
      <type>gaussian</type>
      <mean>0.0</mean>
      <stddev>0.008</stddev>
    </noise>
  </ray>
  <always_on>true</always_on>
  <visualize>true</visualize>
  <update_rate>10</update_rate>
</sensor>
```

## Camera Simulation

Camera sensors are essential for computer vision applications, object recognition, and visual navigation.

### Basic RGB Camera Configuration

```xml
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
```

### Adding Noise to Camera

```xml>
<sensor name="noisy_camera" type="camera">
  <camera name="head_camera">
    <horizontal_fov>1.047</horizontal_fov>
    <image>
      <width>640</width>
      <height>480</height>
      <format>R8G8B8</format>
    </image>
    <clip>
      <near>0.1</near>
      <far>100</far>
    </clip>
    <noise>
      <type>gaussian</type>
      <mean>0.0</mean>
      <stddev>0.007</stddev>
    </noise>
  </camera>
  <always_on>true</always_on>
  <update_rate>30</update_rate>
  <visualize>true</visualize>
</sensor>
```

### Depth Camera Configuration

Depth cameras provide both RGB and depth information:

```xml
<sensor name="depth_camera" type="depth">
  <pose>0.2 0 0.3 0 0 0</pose>
  <camera name="depth_head">
    <horizontal_fov>1.047</horizontal_fov>  <!-- 60 degrees -->
    <image>
      <width>640</width>
      <height>480</height>
      <format>R8G8B8</format>
    </image>
    <depth_camera>
      <output>depths</output>
    </depth_camera>
    <clip>
      <near>0.1</near>
      <far>10</far>
    </clip>
    <noise>
      <type>gaussian</type>
      <mean>0.0</mean>
      <stddev>0.01</stddev>
    </noise>
  </camera>
  <always_on>true</always_on>
  <update_rate>30</update_rate>
  <visualize>true</visualize>
</sensor>
```

## IMU Simulation

Inertial Measurement Units (IMUs) provide information about acceleration and angular velocity, which are crucial for robot state estimation and control.

### Basic IMU Configuration

```xml
<sensor name="imu_sensor" type="imu">
  <always_on>true</always_on>
  <update_rate>100</update_rate>
  <pose>0 0 0.5 0 0 0</pose>
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
```

## GPS Simulation

For outdoor robots, GPS sensors provide global positioning information:

```xml
<sensor name="gps_sensor" type="gps">
  <always_on>true</always_on>
  <update_rate>1</update_rate>
  <pose>0 0 0.5 0 0 0</pose>
  <gps>
    <position_sensing>
      <horizontal>
        <noise type="gaussian">
          <mean>0.0</mean>
          <stddev>2.0</stddev>  <!-- 2 meter standard deviation -->
        </noise>
      </horizontal>
      <vertical>
        <noise type="gaussian">
          <mean>0.0</mean>
          <stddev>4.0</stddev>  <!-- 4 meter standard deviation -->
        </noise>
      </vertical>
    </position_sensing>
    <velocity_sensing>
      <horizontal>
        <noise type="gaussian">
          <mean>0.0</mean>
          <stddev>0.1</stddev>
        </noise>
      </horizontal>
      <vertical>
        <noise type="gaussian">
          <mean>0.0</mean>
          <stddev>0.1</stddev>
        </noise>
      </vertical>
    </velocity_sensing>
  </gps>
</sensor>
```

## Connecting Sensors to ROS 2

To use simulated sensors with ROS 2, we need to attach ROS plugins to our sensors:

### Camera with ROS 2 Interface

```xml
<sensor name="camera" type="camera">
  <camera name="head_camera">
    <horizontal_fov>1.047</horizontal_fov>
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
  
  <!-- ROS 2 interface plugin -->
  <plugin name="camera_controller" filename="gz-sim-camera-system">
    <frame_name>camera_frame</frame_name>
    <topic_name>/camera/image_raw</topic_name>
  </plugin>
</sensor>
```

### LiDAR with ROS 2 Interface

```xml
<sensor name="lidar" type="ray">
  <ray>
    <!-- LiDAR configuration as before -->
  </ray>
  
  <!-- ROS 2 interface plugin -->
  <plugin name="lidar_controller" filename="gz-sim-ray-system">
    <frame_name>lidar_frame</frame_name>
    <topic_name>/scan</topic_name>
  </plugin>
</sensor>
```

## Sensor Fusion in Simulation

Simulating multiple sensors allows us to develop and test sensor fusion algorithms:

### Example: Multi-Sensor Robot Configuration

```xml
<model name="sensor_fusion_robot">
  <!-- Base link -->
  <link name="base_link">
    <inertial>
      <mass>10</mass>
      <inertia>
        <ixx>0.4</ixx>
        <ixy>0</ixy>
        <ixz>0</ixz>
        <iyy>0.4</iyy>
        <iyz>0</iyz>
        <izz>0.4</izz>
      </inertia>
    </inertial>
  </link>

  <!-- IMU sensor -->
  <sensor name="imu" type="imu">
    <pose>0 0 0.1 0 0 0</pose>
    <always_on>true</always_on>
    <update_rate>100</update_rate>
    <!-- IMU configuration as shown earlier -->
  </sensor>

  <!-- Camera sensor -->
  <sensor name="camera" type="camera">
    <pose>0.1 0 0.2 0 0 0</pose>
    <!-- Camera configuration as shown earlier -->
  </sensor>

  <!-- LiDAR sensor -->
  <sensor name="lidar" type="ray">
    <pose>0.1 0 0.3 0 0 0</pose>
    <!-- LiDAR configuration as shown earlier -->
  </sensor>

  <!-- GPS sensor -->
  <sensor name="gps" type="gps">
    <pose>0 0 0.5 0 0 0</pose>
    <!-- GPS configuration as shown earlier -->
  </sensor>
</model>
```

## Validating Sensor Simulation

To ensure your simulated sensors are realistic, compare their output to real sensor data when possible:

### 1. Noise Characteristics
- Analyze the statistical properties of your simulated sensor data
- Compare standard deviation, mean, and distribution to real sensors
- Adjust noise parameters to match real hardware characteristics

### 2. Performance Metrics
- Measure sensor update rates to ensure they match specifications
- Check that sensor ranges and fields of view match specifications
- Verify that sensor data is published to the correct ROS 2 topics

### 3. Environmental Interactions
- Test sensors in various lighting conditions (for cameras)
- Test LiDAR with objects of different materials and reflectivities
- Validate IMU readings during various motion patterns

## Performance Optimization

Sensor simulation can be computationally expensive. Consider these optimizations:

### 1. Update Rates
- Use appropriate update rates for each sensor type
- Lower update rates for less critical sensors
- Consider variable update rates based on robot motion

### 2. Resolution
- Match sensor resolution to actual requirements
- Use lower resolution for distant objects or less critical sensors
- Consider Level of Detail (LOD) approaches

### 3. Visualization
- Disable visualization when running headless
- Use simplified models for collision detection when full detail isn't needed

## Troubleshooting Common Issues

### Sensor Data Issues
- **No data published**: Check that sensors are turned on (`<always_on>true</always_on>`)
- **Incorrect frame IDs**: Verify that frame names match your TF tree
- **Wrong data types**: Ensure your ROS 2 nodes expect the correct message types

### Performance Issues
- **Slow simulation**: Reduce sensor update rates or resolution
- **High CPU usage**: Check that visualizations are disabled when not needed
- **Memory leaks**: Monitor for proper cleanup of sensor data

### Accuracy Issues
- **Inconsistent readings**: Check noise parameters and physics properties
- **Wrong units**: Verify that units match between simulation and processing nodes
- **Timing issues**: Ensure proper time synchronization between components

## Advanced Sensor Simulation

### Custom Sensor Plugins

For specialized sensors, you may need to create custom Gazebo plugins:

```cpp
// Example structure for a custom sensor plugin
#include <gazebo/sensors/Sensor.hh>
#include <gazebo/sensors/SensorTypes.hh>
#include <gazebo/util/system.hh>

class GAZEBO_VISIBLE CustomSensorPlugin : public SensorPlugin
{
  public: void Load(sensors::SensorPtr _sensor, sdf::ElementPtr _sdf)
  {
    // Initialize sensor
    this->parentSensor = 
      std::dynamic_pointer_cast<sensors::RaySensor>(_sensor);

    if (!this->parentSensor)
    {
      gzerr << "CustomSensorPlugin not attached to a ray sensor\n";
      return;
    }

    // Connect to sensor update event
    this->updateConnection = this->parentSensor->ConnectUpdated(
        std::bind(&CustomSensorPlugin::OnUpdate, this));

    // Activate sensor
    this->parentSensor->SetActive(true);
  }

  public: void OnUpdate()
  {
    // Process sensor data
    auto ranges = this->parentSensor->Ranges();
    // Apply custom processing
    // Publish to ROS 2 topic
  }

  private: sensors::RaySensorPtr parentSensor;
  private: event::ConnectionPtr updateConnection;
};
```

## Sensor Simulation for Humanoid Robotics

For humanoid robotics applications, special considerations apply:

### 1. Balance-Related Sensors
- IMUs are critical for balance control
- Proper noise modeling is essential for realistic balance challenges
- Multiple IMUs may be needed for full-body awareness

### 2. Manipulation Sensors
- Force/torque sensors in hands for grasp detection
- Tactile sensors for fine manipulation
- Proprioceptive sensors for joint position feedback

### 3. Environmental Perception
- Stereo vision for depth perception during navigation
- 360-degree LiDAR for full environment awareness
- Multiple cameras for different perspectives

## Best Practices for Sensor Simulation

1. **Match Real Hardware**: Configure sensors to closely match the specifications of your real hardware
2. **Include Noise**: Always include realistic noise models in your simulation
3. **Validate Results**: Compare simulation results with real-world data when possible
4. **Consider Computational Cost**: Balance simulation fidelity with performance requirements
5. **Document Limitations**: Keep track of how your simulation differs from reality

## Summary

In this section, we've explored how to simulate various types of sensors in Gazebo with realistic properties. We've covered configuration options for cameras, LiDAR, IMUs, and other common robot sensors, including how to add noise and connect them to ROS 2. Sensor simulation is a critical component of effective digital twins, allowing for safe and efficient testing of perception and navigation algorithms.

In the next section, we'll explore how to connect these simulated sensors to ROS 2 nodes using the Gazebo Bridge, enabling full integration between simulation and robotic software stacks.