# ROS 2 Introduction

ROS 2 (Robot Operating System 2) is not an operating system but rather a middleware framework that provides services designed for a heterogeneous computer cluster. It includes hardware abstraction, device drivers, libraries, visualizers, message-passing, package management, and more. ROS 2 is designed to be the nervous system of a robot, coordinating the interaction between different components.

## Learning Objectives

By the end of this section, you will be able to:
- Explain the core concepts and architecture of ROS 2
- Identify the differences between ROS 1 and ROS 2
- Understand the role of DDS in ROS 2 communication
- Recognize the main components of a ROS 2 system
- Set up a basic ROS 2 workspace

## What is ROS 2?

ROS 2 is the next generation of the Robot Operating System, designed to address the limitations of ROS 1 while maintaining its strengths. The key improvements include:

- **Real-time support**: Better determinism and real-time capabilities
- **Multi-robot systems**: Improved support for multi-robot scenarios
- **Production deployment**: Better for commercial and industrial applications
- **Security**: Built-in security features
- **DDS-based communication**: Uses Data Distribution Service (DDS) for communication

### Core Concepts

#### Nodes
A node is a process that performs computation. Nodes are the fundamental building blocks of ROS programs. In a humanoid robot system, you might have separate nodes for:
- Perception (computer vision, sensor processing)
- Planning (motion planning, path planning)
- Control (motor control, actuator commands)
- Communication (networking, data logging)

#### Topics
Topics enable asynchronous, many-to-many communication between nodes. Publishers send messages to a topic, and subscribers receive messages from a topic. This pattern is ideal for streaming data like sensor readings or robot joint positions.

#### Services
Services provide synchronous, request-response communication. A client sends a request to a service, and the service returns a response. This pattern is ideal for actions that have a clear start and end, such as requesting a robot to move to a specific location.

#### Actions
Actions are similar to services but for long-running tasks that may take a significant amount of time. Actions provide feedback during execution and can be preempted before completion.

## ROS 2 vs. ROS 1

ROS 2 addresses several limitations of ROS 1:

### Architecture
- **ROS 1**: Centralized master-based architecture (single point of failure)
- **ROS 2**: Decentralized architecture using DDS (more robust and scalable)

### Real-time Support
- **ROS 1**: Limited real-time capabilities
- **ROS 2**: Better real-time support with deterministic behavior

### Security
- **ROS 1**: No built-in security features
- **ROS 2**: Built-in security features including authentication, access control, and encryption

### Multi-robot Systems
- **ROS 1**: Complex setup for multi-robot systems
- **ROS 2**: Native support for multi-robot systems with improved namespace handling

### Middleware
- **ROS 1**: Custom transport layer
- **ROS 2**: DDS (Data Distribution Service) providing standard middleware capabilities

## DDS in ROS 2

DDS (Data Distribution Service) is an OMG (Object Management Group) standard for real-time systems. In ROS 2, DDS handles the underlying communication between nodes. Different DDS implementations can be used with ROS 2, including:

- **Fast DDS** (default in ROS 2 Humble)
- **Cyclone DDS**
- **RTI Connext DDS**

DDS provides:
- **Discovery**: Automatic discovery of participants in the system
- **Transport**: Reliable and best-effort transport options
- **Quality of Service (QoS)**: Configurable policies for different communication requirements

## Main Components of ROS 2

### 1. Client Libraries
ROS 2 provides client libraries for different programming languages:
- **rclcpp**: C++ client library
- **rclpy**: Python client library (our focus for this textbook)
- **rclrs**: Rust client library
- **rclc**: C client library

### 2. Command Line Tools
ROS 2 includes command-line tools for development and debugging:
- `ros2 run`: Run a node
- `ros2 topic`: Interact with topics
- `ros2 service`: Interact with services
- `ros2 action`: Interact with actions
- `ros2 node`: Interact with nodes
- `ros2 param`: Interact with parameters

### 3. Development Tools
- **RViz2**: 3D visualization tool
- **rqt**: Graphical user interface tools
- **rosbag2**: Data recording and playback
- **colcon**: Build system for ROS packages

## Setting Up Your Environment

### Prerequisites
Before starting, ensure you have ROS 2 Humble Hawksbill installed on Ubuntu 22.04. If you haven't installed it yet, follow the official installation guide.

### Creating a Workspace

Let's create a workspace for our humanoid robotics project:

```bash
# Create the workspace directory
mkdir -p ~/humanoid_ws/src
cd ~/humanoid_ws

# Source the ROS 2 installation
source /opt/ros/humble/setup.bash

# Build the workspace (even though it's empty)
colcon build
```

### Environment Setup

To use ROS 2, you need to source the setup file. You can add this to your `~/.bashrc` to automatically source it:

```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
echo "source ~/humanoid_ws/install/setup.bash" >> ~/.bashrc
```

## Basic ROS 2 Concepts in Practice

### Creating a Package

Let's create our first ROS 2 package:

```bash
cd ~/humanoid_ws/src
ros2 pkg create --build-type ament_python humanoid_basics
```

This creates a basic Python package structure. Inside the package, you'll find:
- `setup.py`: Python package configuration
- `setup.cfg`: Installation configuration
- `package.xml`: Package metadata
- `humanoid_basics/`: Python module directory

### A Simple Node

Let's create a simple node that publishes a message:

```python
# File: humanoid_ws/src/humanoid_basics/humanoid_basics/simple_publisher.py

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class SimplePublisher(Node):

    def __init__(self):
        super().__init__('simple_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = f'Hello World: {self.i}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.i += 1


def main(args=None):
    rclpy.init(args=args)

    simple_publisher = SimplePublisher()

    rclpy.spin(simple_publisher)

    # Destroy the node explicitly
    simple_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### Making the Node Executable

To make our node executable, we need to modify the `setup.py` file:

```python
# In humanoid_ws/src/humanoid_basics/setup.py
import os
from glob import glob
from setuptools import setup

package_name = 'humanoid_basics'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your.email@example.com',
    description='Simple ROS 2 publisher example for humanoid robotics',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'simple_publisher = humanoid_basics.simple_publisher:main',
        ],
    },
)
```

### Running the Node

After making these changes, rebuild the package:

```bash
cd ~/humanoid_ws
colcon build --packages-select humanoid_basics
source install/setup.bash
ros2 run humanoid_basics simple_publisher
```

## Quality of Service (QoS) Profiles

QoS profiles in ROS 2 allow you to specify the behavior of topics and services. Key QoS settings include:

- **Reliability**: Whether messages should be reliably delivered
- **Durability**: Whether late-joining subscribers should receive old messages
- **History**: How many messages to store
- **Deadline**: Maximum time between messages

For example, sensor data might use best-effort reliability with keep-last history, while critical commands might use reliable delivery with keep-all history.

## Namespaces

Namespaces in ROS 2 allow you to group related nodes and topics. This is particularly useful in multi-robot systems:

```python
# Example of using namespaces
node = Node('robot_controller', namespace='robot1')
```

This would place the node in the `/robot1` namespace, so its topics would be `/robot1/topic_name` instead of just `/topic_name`.

## Parameters

Parameters in ROS 2 allow you to configure nodes at runtime:

```python
# Declaring a parameter
self.declare_parameter('param_name', 'default_value')

# Getting a parameter value
param_value = self.get_parameter('param_name').value
```

Parameters can be set at launch time, changed during runtime, or loaded from configuration files.

## Summary

In this section, we've covered the fundamental concepts of ROS 2, including nodes, topics, services, and actions. We've also looked at the key differences between ROS 1 and ROS 2, the role of DDS, and the main components of a ROS 2 system.

In the next section, we'll dive deeper into ROS 2 communication patterns by exploring nodes, topics, and services in more detail.