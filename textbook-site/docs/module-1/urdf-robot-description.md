# URDF Robot Description

URDF (Unified Robot Description Format) is an XML-based format used in ROS to describe robot models. URDF is essential for robotics applications as it defines the physical structure, visual representation, and kinematic properties of robots. In this section, we'll explore how to create and work with URDF files for humanoid robots.

## Learning Objectives

By the end of this section, you will be able to:
- Create URDF files for complex robots with multiple joints and links
- Define visual, collision, and inertial properties for robot components
- Implement proper kinematic chains for humanoid robots
- Use Xacro to simplify complex URDF definitions
- Validate URDF files for correctness
- Integrate URDF with Gazebo simulation
- Apply URDF best practices for humanoid robotics

## Introduction to URDF

URDF is an XML format that describes the physical properties of a robot, including its links, joints, and other components. A URDF file contains information about the robot's:

- Physical structure (links and joints)
- Visual representation (shapes, colors, materials)
- Collision properties (shapes for collision detection)
- Inertial properties (mass, center of mass, moments of inertia)
- Joint limits and kinematic constraints

For humanoid robots, URDF becomes especially important as these robots have many degrees of freedom and complex kinematic structures.

### Basic URDF Structure

A URDF file consists of:

- `<robot>`: The root element that contains the entire robot description
- `<link>`: Represents a rigid body part of the robot
- `<joint>`: Connects two links and defines their relative motion
- `<material>`: Defines colors and textures
- `<gazebo>`: Gazebo-specific extensions for simulation

## Creating a Basic Humanoid Robot URDF

Let's start with a simplified humanoid robot model:

```xml
<?xml version="1.0"?>
<robot name="simple_humanoid">
  <!-- Base/Root Link -->
  <link name="base_link">
    <inertial>
      <mass value="10.0"/>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <inertia ixx="0.4" ixy="0" ixz="0" iyy="0.4" iyz="0" izz="0.4"/>
    </inertial>
    
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <box size="0.3 0.3 0.6"/>
      </geometry>
      <material name="gray">
        <color rgba="0.5 0.5 0.5 1"/>
      </material>
    </visual>
    
    <collision>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <box size="0.3 0.3 0.6"/>
      </geometry>
    </collision>
  </link>

  <!-- Torso -->
  <link name="torso">
    <inertial>
      <mass value="8.0"/>
      <origin xyz="0 0 0.3" rpy="0 0 0"/>
      <inertia ixx="0.6" ixy="0" ixz="0" iyy="0.6" iyz="0" izz="0.2"/>
    </inertial>
    
    <visual>
      <origin xyz="0 0 0.3" rpy="0 0 0"/>
      <geometry>
        <box size="0.25 0.25 0.6"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 1 1"/>
      </material>
    </visual>
    
    <collision>
      <origin xyz="0 0 0.3" rpy="0 0 0"/>
      <geometry>
        <box size="0.25 0.25 0.6"/>
      </geometry>
    </collision>
  </link>

  <!-- Neck Joint -->
  <joint name="neck_joint" type="revolute">
    <parent link="torso"/>
    <child link="neck"/>
    <origin xyz="0 0 0.6" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>  <!-- Y-axis rotation for pitch -->
    <limit lower="-0.785" upper="0.785" effort="100" velocity="1"/>
  </joint>

  <!-- Neck Link -->
  <link name="neck">
    <inertial>
      <mass value="0.5"/>
      <origin xyz="0 0 0.05" rpy="0 0 0"/>
      <inertia ixx="0.001" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.001"/>
    </inertial>
    
    <visual>
      <origin xyz="0 0 0.05" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.05" length="0.1"/>
      </geometry>
      <material name="white">
        <color rgba="1 1 1 1"/>
      </material>
    </visual>
    
    <collision>
      <origin xyz="0 0 0.05" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.05" length="0.1"/>
      </geometry>
    </collision>
  </link>

  <!-- Head -->
  <joint name="head_joint" type="revolute">
    <parent link="neck"/>
    <child link="head"/>
    <origin xyz="0 0 0.1" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>  <!-- Z-axis rotation for yaw -->
    <limit lower="-1.57" upper="1.57" effort="50" velocity="1"/>
  </joint>

  <link name="head">
    <inertial>
      <mass value="2.0"/>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <inertia ixx="0.02" ixy="0" ixz="0" iyy="0.02" iyz="0" izz="0.02"/>
    </inertial>
    
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <sphere radius="0.1"/>
      </geometry>
      <material name="skin">
        <color rgba="1 0.8 0.6 1"/>
      </material>
    </visual>
    
    <collision>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <sphere radius="0.1"/>
      </geometry>
    </collision>
  </link>

  <!-- Left Shoulder -->
  <joint name="left_shoulder_pan_joint" type="revolute">
    <parent link="torso"/>
    <child link="left_shoulder"/>
    <origin xyz="0.15 0.15 0.3" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>  <!-- Z-axis for shoulder rotation -->
    <limit lower="-1.57" upper="1.57" effort="100" velocity="1"/>
  </joint>

  <link name="left_shoulder">
    <inertial>
      <mass value="1.0"/>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/>
    </inertial>
    
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <sphere radius="0.05"/>
      </geometry>
      <material name="red">
        <color rgba="1 0 0 1"/>
      </material>
    </visual>
    
    <collision>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <sphere radius="0.05"/>
      </geometry>
    </collision>
  </link>

  <!-- Left Upper Arm -->
  <joint name="left_shoulder_lift_joint" type="revolute">
    <parent link="left_shoulder"/>
    <child link="left_upper_arm"/>
    <origin xyz="0 0.1 0" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>  <!-- Y-axis for lifting motion -->
    <limit lower="-1.57" upper="1.57" effort="100" velocity="1"/>
  </joint>

  <link name="left_upper_arm">
    <inertial>
      <mass value="1.5"/>
      <origin xyz="0 0 -0.15" rpy="0 0 0"/>
      <inertia ixx="0.02" ixy="0" ixz="0" iyy="0.02" iyz="0" izz="0.005"/>
    </inertial>
    
    <visual>
      <origin xyz="0 0 -0.15" rpy="0 0 0"/>
      <geometry>
        <capsule radius="0.04" length="0.2"/>
      </geometry>
      <material name="red">
        <color rgba="1 0 0 1"/>
      </material>
    </visual>
    
    <collision>
      <origin xyz="0 0 -0.15" rpy="0 0 0"/>
      <geometry>
        <capsule radius="0.04" length="0.2"/>
      </geometry>
    </collision>
  </link>

  <!-- Continue with more joints and links for the rest of the humanoid -->
  <!-- Left Elbow -->
  <joint name="left_elbow_joint" type="revolute">
    <parent link="left_upper_arm"/>
    <child link="left_lower_arm"/>
    <origin xyz="0 0 -0.3" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>  <!-- Z-axis for elbow rotation -->
    <limit lower="0" upper="2.356" effort="50" velocity="1"/>
  </joint>

  <link name="left_lower_arm">
    <inertial>
      <mass value="1.0"/>
      <origin xyz="0 0 -0.1" rpy="0 0 0"/>
      <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.003"/>
    </inertial>
    
    <visual>
      <origin xyz="0 0 -0.1" rpy="0 0 0"/>
      <geometry>
        <capsule radius="0.03" length="0.14"/>
      </geometry>
      <material name="red">
        <color rgba="1 0 0 1"/>
      </material>
    </visual>
    
    <collision>
      <origin xyz="0 0 -0.1" rpy="0 0 0"/>
      <geometry>
        <capsule radius="0.03" length="0.14"/>
      </geometry>
    </collision>
  </link>

  <!-- Left Hand -->
  <joint name="left_wrist_joint" type="revolute">
    <parent link="left_lower_arm"/>
    <child link="left_hand"/>
    <origin xyz="0 0 -0.2" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>  <!-- Y-axis for wrist movement -->
    <limit lower="-1.57" upper="1.57" effort="20" velocity="1"/>
  </joint>

  <link name="left_hand">
    <inertial>
      <mass value="0.3"/>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <inertia ixx="0.001" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.001"/>
    </inertial>
    
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <sphere radius="0.03"/>
      </geometry>
      <material name="skin">
        <color rgba="1 0.8 0.6 1"/>
      </material>
    </visual>
    
    <collision>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <sphere radius="0.03"/>
      </geometry>
    </collision>
  </link>

  <!-- Right Arm (similar to left arm) -->
  <joint name="right_shoulder_pan_joint" type="revolute">
    <parent link="torso"/>
    <child link="right_shoulder"/>
    <origin xyz="0.15 -0.15 0.3" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-1.57" upper="1.57" effort="100" velocity="1"/>
  </joint>

  <link name="right_shoulder">
    <inertial>
      <mass value="1.0"/>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/>
    </inertial>
    
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <sphere radius="0.05"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 1 1"/>
      </material>
    </visual>
    
    <collision>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <sphere radius="0.05"/>
      </geometry>
    </collision>
  </link>

  <joint name="right_shoulder_lift_joint" type="revolute">
    <parent link="right_shoulder"/>
    <child link="right_upper_arm"/>
    <origin xyz="0 -0.1 0" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="100" velocity="1"/>
  </joint>

  <link name="right_upper_arm">
    <inertial>
      <mass value="1.5"/>
      <origin xyz="0 0 -0.15" rpy="0 0 0"/>
      <inertia ixx="0.02" ixy="0" ixz="0" iyy="0.02" iyz="0" izz="0.005"/>
    </inertial>
    
    <visual>
      <origin xyz="0 0 -0.15" rpy="0 0 0"/>
      <geometry>
        <capsule radius="0.04" length="0.2"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 1 1"/>
      </material>
    </visual>
    
    <collision>
      <origin xyz="0 0 -0.15" rpy="0 0 0"/>
      <geometry>
        <capsule radius="0.04" length="0.2"/>
      </geometry>
    </collision>
  </link>

  <joint name="right_elbow_joint" type="revolute">
    <parent link="right_upper_arm"/>
    <child link="right_lower_arm"/>
    <origin xyz="0 0 -0.3" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="0" upper="2.356" effort="50" velocity="1"/>
  </joint>

  <link name="right_lower_arm">
    <inertial>
      <mass value="1.0"/>
      <origin xyz="0 0 -0.1" rpy="0 0 0"/>
      <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.003"/>
    </inertial>
    
    <visual>
      <origin xyz="0 0 -0.1" rpy="0 0 0"/>
      <geometry>
        <capsule radius="0.03" length="0.14"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 1 1"/>
      </material>
    </visual>
    
    <collision>
      <origin xyz="0 0 -0.1" rpy="0 0 0"/>
      <geometry>
        <capsule radius="0.03" length="0.14"/>
      </geometry>
    </collision>
  </link>

  <joint name="right_wrist_joint" type="revolute">
    <parent link="right_lower_arm"/>
    <child link="right_hand"/>
    <origin xyz="0 0 -0.2" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="20" velocity="1"/>
  </joint>

  <link name="right_hand">
    <inertial>
      <mass value="0.3"/>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <inertia ixx="0.001" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.001"/>
    </inertial>
    
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <sphere radius="0.03"/>
      </geometry>
      <material name="skin">
        <color rgba="1 0.8 0.6 1"/>
      </material>
    </visual>
    
    <collision>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <sphere radius="0.03"/>
      </geometry>
    </collision>
  </link>

  <!-- Left Hip Joint -->
  <joint name="left_hip_joint" type="revolute">
    <parent link="base_link"/>
    <child link="left_thigh"/>
    <origin xyz="-0.1 0.1 -0.3" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>  <!-- Z-axis for hip rotation -->
    <limit lower="-1.57" upper="1.57" effort="200" velocity="1"/>
  </joint>

  <link name="left_thigh">
    <inertial>
      <mass value="3.0"/>
      <origin xyz="0 0 -0.2" rpy="0 0 0"/>
      <inertia ixx="0.05" ixy="0" ixz="0" iyy="0.05" iyz="0" izz="0.01"/>
    </inertial>
    
    <visual>
      <origin xyz="0 0 -0.2" rpy="0 0 0"/>
      <geometry>
        <capsule radius="0.06" length="0.36"/>
      </geometry>
      <material name="green">
        <color rgba="0 1 0 1"/>
      </material>
    </visual>
    
    <collision>
      <origin xyz="0 0 -0.2" rpy="0 0 0"/>
      <geometry>
        <capsule radius="0.06" length="0.36"/>
      </geometry>
    </collision>
  </link>

  <!-- Left Knee Joint -->
  <joint name="left_knee_joint" type="revolute">
    <parent link="left_thigh"/>
    <child link="left_shin"/>
    <origin xyz="0 0 -0.4" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>  <!-- Y-axis for knee flexion -->
    <limit lower="0" upper="2.356" effort="200" velocity="1"/>  <!-- Knee only bends one way -->
  </joint>

  <link name="left_shin">
    <inertial>
      <mass value="2.5"/>
      <origin xyz="0 0 -0.18" rpy="0 0 0"/>
      <inertia ixx="0.03" ixy="0" ixz="0" iyy="0.03" iyz="0" izz="0.008"/>
    </inertial>
    
    <visual>
      <origin xyz="0 0 -0.18" rpy="0 0 0"/>
      <geometry>
        <capsule radius="0.05" length="0.32"/>
      </geometry>
      <material name="green">
        <color rgba="0 1 0 1"/>
      </material>
    </visual>
    
    <collision>
      <origin xyz="0 0 -0.18" rpy="0 0 0"/>
      <geometry>
        <capsule radius="0.05" length="0.32"/>
      </geometry>
    </collision>
  </link>

  <!-- Left Ankle Joint -->
  <joint name="left_ankle_joint" type="revolute">
    <parent link="left_shin"/>
    <child link="left_foot"/>
    <origin xyz="0 0 -0.36" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>  <!-- Z-axis for foot rotation -->
    <limit lower="-0.52" upper="0.52" effort="100" velocity="1"/>
  </joint>

  <link name="left_foot">
    <inertial>
      <mass value="1.0"/>
      <origin xyz="0.05 0 0" rpy="0 0 0"/>
      <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/>
    </inertial>
    
    <visual>
      <origin xyz="0.05 0 0" rpy="0 0 0"/>
      <geometry>
        <box size="0.15 0.08 0.05"/>
      </geometry>
      <material name="black">
        <color rgba="0 0 0 1"/>
      </material>
    </visual>
    
    <collision>
      <origin xyz="0.05 0 0" rpy="0 0 0"/>
      <geometry>
        <box size="0.15 0.08 0.05"/>
      </geometry>
    </collision>
  </link>

  <!-- Right Leg (similar to left leg) -->
  <joint name="right_hip_joint" type="revolute">
    <parent link="base_link"/>
    <child link="right_thigh"/>
    <origin xyz="-0.1 -0.1 -0.3" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-1.57" upper="1.57" effort="200" velocity="1"/>
  </joint>

  <link name="right_thigh">
    <inertial>
      <mass value="3.0"/>
      <origin xyz="0 0 -0.2" rpy="0 0 0"/>
      <inertia ixx="0.05" ixy="0" ixz="0" iyy="0.05" iyz="0" izz="0.01"/>
    </inertial>
    
    <visual>
      <origin xyz="0 0 -0.2" rpy="0 0 0"/>
      <geometry>
        <capsule radius="0.06" length="0.36"/>
      </geometry>
      <material name="yellow">
        <color rgba="1 1 0 1"/>
      </material>
    </visual>
    
    <collision>
      <origin xyz="0 0 -0.2" rpy="0 0 0"/>
      <geometry>
        <capsule radius="0.06" length="0.36"/>
      </geometry>
    </collision>
  </link>

  <joint name="right_knee_joint" type="revolute">
    <parent link="right_thigh"/>
    <child link="right_shin"/>
    <origin xyz="0 0 -0.4" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="0" upper="2.356" effort="200" velocity="1"/>
  </joint>

  <link name="right_shin">
    <inertial>
      <mass value="2.5"/>
      <origin xyz="0 0 -0.18" rpy="0 0 0"/>
      <inertia ixx="0.03" ixy="0" ixz="0" iyy="0.03" iyz="0" izz="0.008"/>
    </inertial>
    
    <visual>
      <origin xyz="0 0 -0.18" rpy="0 0 0"/>
      <geometry>
        <capsule radius="0.05" length="0.32"/>
      </geometry>
      <material name="yellow">
        <color rgba="1 1 0 1"/>
      </material>
    </visual>
    
    <collision>
      <origin xyz="0 0 -0.18" rpy="0 0 0"/>
      <geometry>
        <capsule radius="0.05" length="0.32"/>
      </geometry>
    </collision>
  </link>

  <joint name="right_ankle_joint" type="revolute">
    <parent link="right_shin"/>
    <child link="right_foot"/>
    <origin xyz="0 0 -0.36" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-0.52" upper="0.52" effort="100" velocity="1"/>
  </joint>

  <link name="right_foot">
    <inertial>
      <mass value="1.0"/>
      <origin xyz="0.05 0 0" rpy="0 0 0"/>
      <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/>
    </inertial>
    
    <visual>
      <origin xyz="0.05 0 0" rpy="0 0 0"/>
      <geometry>
        <box size="0.15 0.08 0.05"/>
      </geometry>
      <material name="black">
        <color rgba="0 0 0 1"/>
      </material>
    </visual>
    
    <collision>
      <origin xyz="0.05 0 0" rpy="0 0 0"/>
      <geometry>
        <box size="0.15 0.08 0.05"/>
      </geometry>
    </collision>
  </link>
</robot>
```

## Using Xacro for Complex URDFs

Xacro (XML Macros) is a macro language that allows you to create more complex URDF files by using variables, constants, and macros. This is especially useful for humanoid robots which have many repeated components:

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="humanoid_with_xacro">

  <!-- Constants -->
  <xacro:property name="M_PI" value="3.1415926535897931" />
  <xacro:property name="PI_OVER_2" value="${M_PI/2}" />
  <xacro:property name="PI_OVER_4" value="${M_PI/4}" />

  <!-- Materials -->
  <material name="blue">
    <color rgba="0 0 1 1"/>
  </material>
  <material name="red">
    <color rgba="1 0 0 1"/>
  </material>
  <material name="green">
    <color rgba="0 1 0 1"/>
  </material>
  <material name="yellow">
    <color rgba="1 1 0 1"/>
  </material>
  <material name="black">
    <color rgba="0 0 0 1"/>
  </material>
  <material name="white">
    <color rgba="1 1 1 1"/>
  </material>
  <material name="gray">
    <color rgba="0.5 0.5 0.5 1"/>
  </material>

  <!-- Macro for creating a limb -->
  <xacro:macro name="limb" params="side parent_link position orientation color">
    <!-- Shoulder -->
    <joint name="${side}_shoulder_joint" type="revolute">
      <parent link="${parent_link}"/>
      <child link="${side}_shoulder"/>
      <origin xyz="${position}" rpy="${orientation}"/>
      <axis xyz="0 0 1"/>
      <limit lower="-${PI_OVER_2}" upper="${PI_OVER_2}" effort="100" velocity="1"/>
    </joint>

    <link name="${side}_shoulder">
      <inertial>
        <mass value="1.0"/>
        <origin xyz="0 0 0" rpy="0 0 0"/>
        <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/>
      </inertial>
      <visual>
        <origin xyz="0 0 0" rpy="0 0 0"/>
        <geometry>
          <sphere radius="0.05"/>
        </geometry>
        <material name="${color}"/>
      </visual>
      <collision>
        <origin xyz="0 0 0" rpy="0 0 0"/>
        <geometry>
          <sphere radius="0.05"/>
        </geometry>
      </collision>
    </link>

    <!-- Upper Arm -->
    <joint name="${side}_upper_arm_joint" type="revolute">
      <parent link="${side}_shoulder"/>
      <child link="${side}_upper_arm"/>
      <origin xyz="0 ${sign(side) * 0.1} 0" rpy="0 0 0"/>
      <axis xyz="0 1 0"/>
      <limit lower="-${PI_OVER_2}" upper="${PI_OVER_2}" effort="100" velocity="1"/>
    </joint>

    <link name="${side}_upper_arm">
      <inertial>
        <mass value="1.5"/>
        <origin xyz="0 0 -0.15" rpy="0 0 0"/>
        <inertia ixx="0.02" ixy="0" ixz="0" iyy="0.02" iyz="0" izz="0.005"/>
      </inertial>
      <visual>
        <origin xyz="0 0 -0.15" rpy="0 0 0"/>
        <geometry>
          <capsule radius="0.04" length="0.2"/>
        </geometry>
        <material name="${color}"/>
      </visual>
      <collision>
        <origin xyz="0 0 -0.15" rpy="0 0 0"/>
        <geometry>
          <capsule radius="0.04" length="0.2"/>
        </geometry>
      </collision>
    </link>

    <!-- Lower Arm -->
    <joint name="${side}_lower_arm_joint" type="revolute">
      <parent link="${side}_upper_arm"/>
      <child link="${side}_lower_arm"/>
      <origin xyz="0 0 -0.3" rpy="0 0 0"/>
      <axis xyz="0 0 1"/>
      <limit lower="0" upper="${M_PI * 0.75}" effort="50" velocity="1"/>
    </joint>

    <link name="${side}_lower_arm">
      <inertial>
        <mass value="1.0"/>
        <origin xyz="0 0 -0.1" rpy="0 0 0"/>
        <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.003"/>
      </inertial>
      <visual>
        <origin xyz="0 0 -0.1" rpy="0 0 0"/>
        <geometry>
          <capsule radius="0.03" length="0.14"/>
        </geometry>
        <material name="${color}"/>
      </visual>
      <collision>
        <origin xyz="0 0 -0.1" rpy="0 0 0"/>
        <geometry>
          <capsule radius="0.03" length="0.14"/>
        </geometry>
      </collision>
    </link>

    <!-- Hand -->
    <joint name="${side}_hand_joint" type="revolute">
      <parent link="${side}_lower_arm"/>
      <child link="${side}_hand"/>
      <origin xyz="0 0 -0.2" rpy="0 0 0"/>
      <axis xyz="0 1 0"/>
      <limit lower="-${PI_OVER_2}" upper="${PI_OVER_2}" effort="20" velocity="1"/>
    </joint>

    <link name="${side}_hand">
      <inertial>
        <mass value="0.3"/>
        <origin xyz="0 0 0" rpy="0 0 0"/>
        <inertia ixx="0.001" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.001"/>
      </inertial>
      <visual>
        <origin xyz="0 0 0" rpy="0 0 0"/>
        <geometry>
          <sphere radius="0.03"/>
        </geometry>
        <material name="skin"/>
      </visual>
      <collision>
        <origin xyz="0 0 0" rpy="0 0 0"/>
        <geometry>
          <sphere radius="0.03"/>
        </geometry>
      </collision>
    </link>
  </xacro:macro>

  <!-- Macro for creating a leg -->
  <xacro:macro name="leg" params="side parent_link position orientation color">
    <!-- Hip -->
    <joint name="${side}_hip_joint" type="revolute">
      <parent link="${parent_link}"/>
      <child link="${side}_hip"/>
      <origin xyz="${position}" rpy="${orientation}"/>
      <axis xyz="0 0 1"/>
      <limit lower="-${PI_OVER_2}" upper="${PI_OVER_2}" effort="200" velocity="1"/>
    </joint>

    <link name="${side}_hip">
      <inertial>
        <mass value="1.0"/>
        <origin xyz="0 0 0" rpy="0 0 0"/>
        <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/>
      </inertial>
      <visual>
        <origin xyz="0 0 0" rpy="0 0 0"/>
        <geometry>
          <sphere radius="0.05"/>
        </geometry>
        <material name="${color}"/>
      </visual>
      <collision>
        <origin xyz="0 0 0" rpy="0 0 0"/>
        <geometry>
          <sphere radius="0.05"/>
        </geometry>
      </collision>
    </link>

    <!-- Thigh -->
    <joint name="${side}_thigh_joint" type="revolute">
      <parent link="${side}_hip"/>
      <child link="${side}_thigh"/>
      <origin xyz="0 0 -0.1" rpy="0 0 0"/>
      <axis xyz="0 1 0"/>
      <limit lower="-${PI_OVER_2}" upper="${PI_OVER_2}" effort="200" velocity="1"/>
    </joint>

    <link name="${side}_thigh">
      <inertial>
        <mass value="3.0"/>
        <origin xyz="0 0 -0.2" rpy="0 0 0"/>
        <inertia ixx="0.05" ixy="0" ixz="0" iyy="0.05" iyz="0" izz="0.01"/>
      </inertial>
      <visual>
        <origin xyz="0 0 -0.2" rpy="0 0 0"/>
        <geometry>
          <capsule radius="0.06" length="0.36"/>
        </geometry>
        <material name="${color}"/>
      </visual>
      <collision>
        <origin xyz="0 0 -0.2" rpy="0 0 0"/>
        <geometry>
          <capsule radius="0.06" length="0.36"/>
        </geometry>
      </collision>
    </link>

    <!-- Shin -->
    <joint name="${side}_shin_joint" type="revolute">
      <parent link="${side}_thigh"/>
      <child link="${side}_shin"/>
      <origin xyz="0 0 -0.4" rpy="0 0 0"/>
      <axis xyz="0 1 0"/>
      <limit lower="0" upper="${M_PI * 0.75}" effort="200" velocity="1"/>
    </joint>

    <link name="${side}_shin">
      <inertial>
        <mass value="2.5"/>
        <origin xyz="0 0 -0.18" rpy="0 0 0"/>
        <inertia ixx="0.03" ixy="0" ixz="0" iyy="0.03" iyz="0" izz="0.008"/>
      </inertial>
      <visual>
        <origin xyz="0 0 -0.18" rpy="0 0 0"/>
        <geometry>
          <capsule radius="0.05" length="0.32"/>
        </geometry>
        <material name="${color}"/>
      </visual>
      <collision>
        <origin xyz="0 0 -0.18" rpy="0 0 0"/>
        <geometry>
          <capsule radius="0.05" length="0.32"/>
        </geometry>
      </collision>
    </link>

    <!-- Foot -->
    <joint name="${side}_foot_joint" type="revolute">
      <parent link="${side}_shin"/>
      <child link="${side}_foot"/>
      <origin xyz="0 0 -0.36" rpy="0 0 0"/>
      <axis xyz="0 0 1"/>
      <limit lower="-${M_PI/6}" upper="${M_PI/6}" effort="100" velocity="1"/>
    </joint>

    <link name="${side}_foot">
      <inertial>
        <mass value="1.0"/>
        <origin xyz="0.05 0 0" rpy="0 0 0"/>
        <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/>
      </inertial>
      <visual>
        <origin xyz="0.05 0 0" rpy="0 0 0"/>
        <geometry>
          <box size="0.15 0.08 0.05"/>
        </geometry>
        <material name="black"/>
      </visual>
      <collision>
        <origin xyz="0.05 0 0" rpy="0 0 0"/>
        <geometry>
          <box size="0.15 0.08 0.05"/>
        </geometry>
      </collision>
    </link>
  </xacro:macro>

  <!-- Base Link -->
  <link name="base_link">
    <inertial>
      <mass value="10.0"/>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <inertia ixx="0.4" ixy="0" ixz="0" iyy="0.4" iyz="0" izz="0.4"/>
    </inertial>
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <box size="0.3 0.3 0.6"/>
      </geometry>
      <material name="gray"/>
    </visual>
    <collision>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <box size="0.3 0.3 0.6"/>
      </geometry>
    </collision>
  </link>

  <!-- Torso -->
  <joint name="torso_joint" type="fixed">
    <parent link="base_link"/>
    <child link="torso"/>
    <origin xyz="0 0 0.3" rpy="0 0 0"/>
  </joint>

  <link name="torso">
    <inertial>
      <mass value="8.0"/>
      <origin xyz="0 0 0.3" rpy="0 0 0"/>
      <inertia ixx="0.6" ixy="0" ixz="0" iyy="0.6" iyz="0" izz="0.2"/>
    </inertial>
    <visual>
      <origin xyz="0 0 0.3" rpy="0 0 0"/>
      <geometry>
        <box size="0.25 0.25 0.6"/>
      </geometry>
      <material name="blue"/>
    </visual>
    <collision>
      <origin xyz="0 0 0.3" rpy="0 0 0"/>
      <geometry>
        <box size="0.25 0.25 0.6"/>
      </geometry>
    </collision>
  </link>

  <!-- Neck -->
  <joint name="neck_joint" type="revolute">
    <parent link="torso"/>
    <child link="neck"/>
    <origin xyz="0 0 0.6" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-${PI_OVER_4}" upper="${PI_OVER_4}" effort="100" velocity="1"/>
  </joint>

  <link name="neck">
    <inertial>
      <mass value="0.5"/>
      <origin xyz="0 0 0.05" rpy="0 0 0"/>
      <inertia ixx="0.001" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.001"/>
    </inertial>
    <visual>
      <origin xyz="0 0 0.05" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.05" length="0.1"/>
      </geometry>
      <material name="white"/>
    </visual>
    <collision>
      <origin xyz="0 0 0.05" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.05" length="0.1"/>
      </geometry>
    </collision>
  </link>

  <!-- Head -->
  <joint name="head_joint" type="revolute">
    <parent link="neck"/>
    <child link="head"/>
    <origin xyz="0 0 0.1" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-${M_PI/2}" upper="${M_PI/2}" effort="50" velocity="1"/>
  </joint>

  <link name="head">
    <inertial>
      <mass value="2.0"/>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <inertia ixx="0.02" ixy="0" ixz="0" iyy="0.02" iyz="0" izz="0.02"/>
    </inertial>
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <sphere radius="0.1"/>
      </geometry>
      <material name="skin"/>
    </visual>
    <collision>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <sphere radius="0.1"/>
      </geometry>
    </collision>
  </link>

  <!-- Create arms using the macro -->
  <xacro:limb side="left" parent_link="torso" position="0.15 0.15 0.3" orientation="0 0 0" color="red"/>
  <xacro:limb side="right" parent_link="torso" position="0.15 -0.15 0.3" orientation="0 0 0" color="blue"/>

  <!-- Create legs using the macro -->
  <xacro:leg side="left" parent_link="base_link" position="-0.1 0.1 -0.3" orientation="0 0 0" color="green"/>
  <xacro:leg side="right" parent_link="base_link" position="-0.1 -0.1 -0.3" orientation="0 0 0" color="yellow"/>

  <!-- Gazebo Plugins -->
  <gazebo>
    <plugin name="gazebo_ros_control" filename="libgazebo_ros_control.so">
      <robotNamespace>/humanoid</robotNamespace>
    </plugin>
  </gazebo>

  <!-- Gazebo Materials -->
  <gazebo reference="base_link">
    <material>Gazebo/Gray</material>
  </gazebo>

  <gazebo reference="torso">
    <material>Gazebo/Blue</material>
  </gazebo>

  <gazebo reference="head">
    <material>Gazebo/Yellow</material>
  </gazebo>

  <gazebo reference="left_arm">
    <material>Gazebo/Red</material>
  </gazebo>

  <gazebo reference="right_arm">
    <material>Gazebo/Blue</material>
  </gazebo>

  <gazebo reference="left_leg">
    <material>Gazebo/Green</material>
  </gazebo>

  <gazebo reference="right_leg">
    <material>Gazebo/Yellow</material>
  </gazebo>

</robot>
```

## Validating URDF Files

Validating URDF files is crucial to ensure they are correctly formatted and represent a physically plausible robot:

### 1. Syntax Validation

```bash
# Check if URDF is well-formed XML
xmllint --noout robot.urdf

# Check for Xacro syntax (if using Xacro)
xacro --check-order robot.xacro
```

### 2. Kinematic Validation

```bash
# Load URDF in RViz to check for visual issues
ros2 run rviz2 rviz2

# Use check_urdf to validate the model
check_urdf robot.urdf
```

### 3. Using check_urdf

The `check_urdf` tool provides valuable information about your robot model:

```bash
# Output detailed information about the robot
check_urdf robot.urdf

# This will show:
# - Number of links and joints
# - Joint types and limits
# - Joint chains from root to tips
# - Any errors or warnings
```

## Gazebo Integration

To integrate your URDF with Gazebo for physics simulation, you'll need to add Gazebo-specific elements:

### 1. Transmission Elements

```xml
<!-- Example transmission for a joint -->
<transmission name="left_shoulder_trans">
  <type>transmission_interface/SimpleTransmission</type>
  <joint name="left_shoulder_joint">
    <hardwareInterface>PositionJointInterface</hardwareInterface>
  </joint>
  <actuator name="left_shoulder_motor">
    <hardwareInterface>PositionJointInterface</hardwareInterface>
    <mechanicalReduction>1</mechanicalReduction>
  </actuator>
</transmission>
```

### 2. Gazebo-Specific Properties

```xml
<!-- Gazebo plugin for ROS control -->
<gazebo>
  <plugin name="gazebo_ros_control" filename="libgazebo_ros_control.so">
    <robotNamespace>/humanoid</robotNamespace>
    <robotSimType>gazebo_ros_control/DefaultRobotHWSim</robotSimType>
  </plugin>
</gazebo>

<!-- Gazebo material properties -->
<gazebo reference="left_foot">
  <mu1>0.9</mu1>
  <mu2>0.9</mu2>
  <kp>1000000.0</kp>
  <kd>100.0</kd>
  <material>Gazebo/Black</material>
</gazebo>
```

## Best Practices for Humanoid URDFs

### 1. Proper Mass and Inertia Values
- Calculate masses based on actual hardware specifications
- Use appropriate inertia values (calculate for geometric shapes)
- Ensure CoM (Center of Mass) is correctly positioned

### 2. Joint Limits
- Set realistic joint limits based on hardware capabilities
- Consider safety margins to prevent damage
- Account for soft limits in addition to hard limits

### 3. Collision Geometry
- Use simplified collision geometry for performance
- Ensure collision geometry adequately represents the physical robot
- Pay attention to collision checking between adjacent links

### 4. Visual Geometry
- Use detailed meshes for visual representation
- Ensure visual geometry matches collision geometry where appropriate
- Optimize mesh complexity for rendering performance

## Summary

In this section, we've covered the fundamentals of creating URDF files for humanoid robots, including:

1. Basic structure of URDF files with links and joints
2. Proper definition of visual, collision, and inertial properties
3. Use of Xacro macros to simplify complex humanoid robot definitions
4. Integration with Gazebo for physics simulation
5. Validation techniques for ensuring URDF correctness
6. Best practices for humanoid robotics applications

URDF is a critical component in robotics development, especially for humanoid robots with complex kinematic structures. Properly defined URDF files enable accurate simulation, visualization, and control of robots in both simulated and real environments.

In the next section, we'll explore how to implement these concepts in practice with hands-on exercises.