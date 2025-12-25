# Appendix A: Hardware Setup Guide

This appendix provides detailed instructions for setting up the hardware required for the Physical AI & Humanoid Robotics textbook.

## Required Hardware Components

### Workstation Requirements
- **GPU**: NVIDIA RTX 4080 or higher (recommended RTX 4090 for optimal performance)
- **CPU**: Intel Core i7-13700K or AMD Ryzen 9 7950X
- **RAM**: 64GB DDR5 (32GB minimum)
- **Storage**: 2TB NVMe SSD (for simulation assets and datasets)
- **OS**: Ubuntu 22.04 LTS

### Edge Computing Hardware
- **Platform**: NVIDIA Jetson Orin Nano Developer Kit
- **Memory**: 8GB or 16GB LPDDR5
- **Connectivity**: WiFi 6, Gigabit Ethernet
- **Interfaces**: Multiple GPIO, I2C, SPI, UART ports

### Robot Platform
- **Model**: Unitree Go2 Edu or higher (Go2, G1, or H1)
- **Sensors**: Built-in LiDAR, IMU, camera
- **Connectivity**: WiFi, Ethernet
- **Battery**: Sufficient runtime for development sessions

## Workstation Setup

### 1. Installing Ubuntu 22.04 LTS

1. Download Ubuntu 22.04 LTS ISO from the official website
2. Create a bootable USB drive using Rufus (Windows) or dd (Linux/MacOS)
3. Boot from USB and follow the installation wizard
4. Select "Normal installation" with third-party software for graphics and Wi-Fi hardware
5. During installation, ensure at least 100GB of disk space is allocated for the project

### 2. Installing NVIDIA Drivers and CUDA

```bash
# Add graphics drivers PPA
sudo add-apt-repository ppa:graphics-drivers/ppa
sudo apt update

# Install NVIDIA drivers (version 535 or higher)
sudo apt install nvidia-driver-535 nvidia-utils-535

# Install CUDA toolkit
wget https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2204/x86_64/cuda-keyring_1.0-1_all.deb
sudo dpkg -i cuda-keyring_1.0-1_all.deb
sudo apt-get update
sudo apt-get -y install cuda-toolkit-12-3
```

### 3. Installing ROS 2 Humble Hawksbill

```bash
# Set locale
locale  # check for UTF-8
sudo apt update && sudo apt install locales
sudo locale-gen en_US.UTF-8

# Add ROS 2 apt repository
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl gnupg lsb-release

sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update
sudo apt upgrade

# Install ROS 2 Humble packages
sudo apt install ros-humble-desktop
sudo apt install ros-humble-ros-base
sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-build-essential
```

### 4. Installing Isaac Sim Prerequisites

```bash
# Install additional dependencies
sudo apt install cmake libeigen3-dev python3-dev python3-pip python3-colcon-common-extensions

# Install Isaac Sim
# Visit https://developer.nvidia.com/isaac-sim and follow the installation guide
# Typically involves downloading Omniverse Launcher and installing Isaac Sim through it
```

## Edge Device Setup (NVIDIA Jetson)

### 1. Flashing the Jetson Orin Nano

1. Download the NVIDIA SDK Manager
2. Connect your Jetson to your host PC
3. Use SDK Manager to flash the Jetson with JetPack (includes ROS 2, CUDA, etc.)

### 2. Installing Isaac ROS on Jetson

```bash
# Add NVIDIA repository
sudo apt update && sudo apt install wget
sudo apt-key adv --fetch-keys https://repo.download.nvidia.com,9550C7D9FE2A8B80.pub
sudo add-apt-repository "deb https://repo.download.nvidia.com/ $(lsb_release -cs) main"

# Install Isaac ROS packages
sudo apt update
sudo apt install ros-humble-isaac-ros-common
sudo apt install ros-humble-isaac-ros-gxf
sudo apt install ros-humble-isaac-ros-image-transport
sudo apt install ros-humble-isaac-ros-apriltag
sudo apt install ros-humble-isaac-ros-bitbots
sudo apt install ros-humble-isaac-ros-dnn-inference
sudo apt install ros-humble-isaac-ros-people-segmentation
sudo apt install ros-humble-isaac-ros-realsense
sudo apt install ros-humble-isaac-ros-visual-slam
```

## Robot Platform Setup (Unitree Go2 Edu)

### 1. Initial Robot Setup

1. Charge the robot battery fully
2. Power on the robot and ensure LED indicators show normal status
3. Connect to the robot's WiFi network
4. Install Unitree SDK following the official documentation

### 2. Network Configuration

```bash
# Connect to robot's WiFi network
# The robot typically creates a network like "Go2-XXXX" where XXXX is the robot's ID

# Configure static IP for consistent connection
sudo nano /etc/netplan/01-network-manager-all.yaml
```

Add the following configuration:

```yaml
network:
  version: 2
  renderer: networkd
  ethernets:
    enp0s3:  # Adjust interface name as needed
      dhcp4: false
      addresses:
        - 192.168.123.100/24
      gateway4: 192.168.123.1
      nameservers:
        addresses: [8.8.8.8, 8.8.4.4]
```

Apply the configuration:

```bash
sudo netplan apply
```

### 3. Testing Hardware Connections

```bash
# Test camera connection
v4l2-ctl --list-devices

# Test network connectivity to robot
ping 192.168.123.1  # Adjust IP as needed

# Test ROS 2 communication
source /opt/ros/humble/setup.bash
ros2 topic list
```

## Troubleshooting Common Hardware Issues

### GPU Not Detected
- Check that power cables are properly connected
- Verify that PCIe cable is properly seated
- Update BIOS to the latest version
- Check that power supply has sufficient wattage (1000W+ recommended)

### Robot WiFi Connection Issues
- Ensure robot is powered on and in correct mode
- Check for interference from other WiFi networks
- Try connecting manually using nmcli
- Verify that firewall isn't blocking robot communication

### Isaac Sim Performance Issues
- Ensure graphics drivers are up to date
- Check that Isaac Sim is using the dedicated GPU
- Close other GPU-intensive applications
- Verify sufficient RAM is available

## Safety Considerations

### Physical Safety
- Always maintain clear space around the robot during operation
- Ensure robot is on stable, level surface
- Keep loose cables away from robot movement area
- Have emergency stop procedure ready

### Electrical Safety
- Use proper grounding for all equipment
- Use surge protectors for expensive hardware
- Ensure power cables are not damaged
- Keep liquids away from electronics

## Maintenance Schedule

### Daily
- Check for proper operation of all systems
- Verify that all connections are secure

### Weekly
- Update software packages
- Check disk space and clean temporary files
- Review system logs for errors

### Monthly
- Update NVIDIA drivers and CUDA
- Update ROS 2 packages
- Check hardware for wear and tear
- Verify backup systems are working

Following this hardware setup guide will ensure you have the proper foundation for implementing all the concepts covered in the Physical AI & Humanoid Robotics textbook.