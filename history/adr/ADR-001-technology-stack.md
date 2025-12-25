# ADR-001: Physical AI & Humanoid Robotics Textbook Technology Stack

**Status**: Accepted  
**Date**: 2025-01-08

## Context

The Physical AI & Humanoid Robotics textbook requires a comprehensive technology stack that integrates multiple complex systems: ROS 2 for robotics control, simulation environments (Gazebo and NVIDIA Isaac Sim), and AI frameworks for embodied intelligence. The technology stack must support hands-on learning experiences while maintaining compatibility with specified hardware (RTX workstation, Jetson Orin Nano, Unitree Go2 Edu).

## Decision

We will use the following integrated technology stack:

- **Documentation Platform**: Docusaurus for textbook delivery
- **Robotics Framework**: ROS 2 Humble Hawksbill
- **Simulation**: Gazebo Garden and NVIDIA Isaac Sim
- **AI Integration**: Python-based AI libraries with OpenAI integration
- **Operating System**: Ubuntu 22.04 LTS
- **Hardware Interface**: rclpy for Python-ROS integration
- **Navigation**: Nav2 for path planning
- **Voice Processing**: OpenAI Whisper for voice commands

## Alternatives Considered

1. **Alternative Stack**: ROS 1 with custom simulation environment
   - Pros: More mature ecosystem, extensive documentation
   - Cons: No longer supported, lacks modern features, not aligned with industry trends

2. **Alternative Stack**: Custom robotics framework with Unity simulation
   - Pros: More flexible, potentially better graphics performance
   - Cons: Requires developing basic robotics functionality, lacks industry standardization, steeper learning curve

3. **Alternative Stack**: ROS 2 with different simulation (Webots, PyBullet)
   - Pros: Potentially lighter weight simulation
   - Cons: Less industry alignment, reduced hardware compatibility, limited photorealistic capabilities

## Consequences

**Positive:**
- Industry-standard technologies that align with current robotics development practices
- Comprehensive simulation capabilities with both physics and photorealistic rendering
- Strong hardware compatibility with specified target platforms
- Rich ecosystem of tools and community support
- Enables simulation-to-reality transfer learning

**Negative:**
- Complex technology stack requiring significant expertise
- Resource-intensive simulation environments
- Potential hardware compatibility issues during validation
- Steep learning curve for students not familiar with ROS 2

## References

- plan.md: Technical Context and Key Dependencies sections
- research.md: ROS 2 Humble Best Practices and Simulation Integration Patterns
- spec.md: Technology stack constraints and hardware focus requirements