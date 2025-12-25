# Appendix B: Troubleshooting Guide

This appendix provides solutions to common issues encountered when implementing the Physical AI & Humanoid Robotics textbook concepts. The guide is organized by component and includes diagnostic procedures and solutions.

## Common Issues by Component

### 1. ROS 2 Issues

#### Problem: ROS 2 nodes not communicating between each other
**Symptoms**: 
- Nodes can't see each other on the network
- Topics are not being published/subscribed properly
- Services return "service not available" errors

**Diagnosis**:
1. Check if nodes are on the same ROS domain:
   ```bash
   echo $ROS_DOMAIN_ID
   ```
2. Verify that RMW implementation is consistent:
   ```bash
   printenv | grep RMW
   ```
3. Check if firewalls are blocking DDS communication

**Solutions**:
1. Set the same domain ID for all nodes:
   ```bash
   export ROS_DOMAIN_ID=0
   ```
2. Ensure all nodes use the same RMW implementation:
   ```bash
   export RMW_IMPLEMENTATION=rmw_cyclonedx_cpp
   ```
3. If using multiple machines, set the same domain ID on all machines
4. Check firewall settings to allow DDS traffic (ports 11811-11911)

#### Problem: High CPU usage with ROS 2
**Symptoms**:
- Nodes consuming excessive CPU resources
- System becoming unresponsive
- Slow message processing

**Solutions**:
1. Reduce QoS settings for non-critical topics
2. Lower update rates for sensors that don't need high frequency
3. Use smaller queue sizes for subscriptions
4. Consider using Cyclone DDS instead of Fast DDS for better performance

### 2. Gazebo Simulation Issues

#### Problem: Gazebo not launching or crashing immediately
**Symptoms**:
- Gazebo closes immediately after opening
- Error messages about OpenGL or graphics driver issues
- Segmentation faults

**Diagnosis**:
1. Check if NVIDIA drivers are properly installed:
   ```bash
   nvidia-smi
   ```
2. Verify OpenGL capabilities:
   ```bash
   glxinfo | grep "OpenGL version"
   ```
3. Check for GPU memory issues

**Solutions**:
1. Update NVIDIA drivers to the latest version
2. Run Gazebo with software rendering:
   ```bash
   export LIBGL_ALWAYS_SOFTWARE=1
   gz sim
   ```
3. Increase GPU memory allocation in BIOS settings
4. Check that you have sufficient VRAM (minimum 8GB recommended)

#### Problem: Physics simulation behaving unrealistically
**Symptoms**:
- Objects falling through the ground
- Jittery or unstable movements
- Robot not responding properly to commands

**Solutions**:
1. Check inertial properties of models:
   - Ensure mass values are realistic
   - Verify that center of mass is properly positioned
   - Check that inertia tensors are properly calculated
2. Adjust physics parameters in world file:
   ```xml
   <physics type="ode">
     <max_step_size>0.001</max_step_size>  <!-- Smaller for more accuracy -->
     <real_time_factor>1</real_time_factor>
     <real_time_update_rate>1000</real_time_update_rate>
   </physics>
   ```
3. Verify collision geometries match visual geometries
4. Check joint limits and constraints

### 3. Isaac Sim Issues

#### Problem: Isaac Sim not launching or very slow
**Symptoms**:
- Isaac Sim takes a very long time to start
- Interface is unresponsive
- Rendering is extremely slow

**Diagnosis**:
1. Check if RTX GPU is being used:
   ```bash
   nvidia-smi
   ```
2. Verify Isaac Sim installation:
   ```bash
   python -c "import omni; print('Isaac Sim modules available')"
   ```
3. Check for CUDA compatibility

**Solutions**:
1. Ensure RTX GPU is set as primary rendering device
2. Update to latest NVIDIA drivers and CUDA toolkit
3. Check that Isaac Sim requirements are met (VRAM, compute capability)
4. Close other GPU-intensive applications
5. Verify that Isaac Sim is not running in compatibility mode

#### Problem: Isaac ROS bridge not working
**Symptoms**:
- No messages being published from Isaac Sim to ROS 2
- Isaac Sim nodes not responding to ROS 2 commands
- Bridge connection errors

**Solutions**:
1. Check that Isaac ROS extensions are enabled in Isaac Sim
2. Verify that ROS 2 network is properly configured
3. Ensure Isaac ROS packages are properly installed:
   ```bash
   ros2 run isaac_ros_apriltag apriltag_node
   ```
4. Check for namespace mismatches between Isaac Sim and ROS 2

### 4. Voice Processing Issues

#### Problem: Whisper STT not recognizing speech properly
**Symptoms**:
- Low accuracy in speech-to-text conversion
- Commands not being understood
- High latency in processing

**Solutions**:
1. Check audio input quality:
   - Verify microphone is properly connected
   - Test with simple audio recording tools
   - Check for background noise
2. Verify OpenAI API key is set:
   ```bash
   echo $OPENAI_API_KEY
   ```
3. Consider using local STT alternatives for better performance
4. Ensure audio format matches Whisper requirements (16kHz, mono, WAV/MP3)

#### Problem: LLM cognitive planning producing irrelevant responses
**Symptoms**:
- LLM generates responses unrelated to robot commands
- Task decomposition doesn't match robot capabilities
- Generated plans are impossible to execute

**Solutions**:
1. Improve prompt engineering with better context and examples
2. Use function calling to constrain LLM responses to valid actions
3. Implement response validation to filter inappropriate outputs
4. Fine-tune the model with robotics-specific examples

### 5. Navigation Issues

#### Problem: Robot getting stuck or not navigating properly
**Symptoms**:
- Robot stops moving unexpectedly
- Navigation planner fails to find path
- Robot oscillates near goal

**Diagnosis**:
1. Check costmap visualization in RViz2
2. Verify that map is properly loaded
3. Check that localization is working

**Solutions**:
1. Adjust costmap inflation parameters:
   ```yaml
   inflation_layer:
     inflation_radius: 0.55  # Increase if robot is getting too close to obstacles
     cost_scaling_factor: 3.0
   ```
2. Verify that robot footprint is properly configured
3. Check that sensor data is properly transforming to base frame
4. Adjust DWA or TEB parameters for smoother navigation

#### Problem: Nav2 path planning failing
**Symptoms**:
- Global planner not finding valid path
- Local planner not executing planned path
- Frequent recovery behaviors

**Solutions**:
1. Verify map quality and resolution
2. Check that robot's maximum velocities are realistic
3. Adjust planner parameters for your specific robot:
   ```yaml
   # In controller server configuration
   WaypointFollower:
     stop_on_failure: false
     waypoint_task_executor_plugin: "wait_at_waypoint"
     wait_at_waypoint:
       enabled: true
       wait_time: 2.0  # seconds
   ```
4. Ensure proper TF tree is established

### 6. Manipulation Issues

#### Problem: Robot arm not reaching desired positions
**Symptoms**:
- IK solver failing frequently
- Robot arm moving to unexpected positions
- Joint limits being violated

**Solutions**:
1. Verify URDF joint limits are correct
2. Check that target poses are within robot's reachable workspace
3. Adjust IK solver parameters
4. Verify that collision objects are properly defined

#### Problem: Grasping failures
**Symptoms**:
- Robot fails to grasp objects consistently
- Gripper approaching object incorrectly
- Object slipping during grasp

**Solutions**:
1. Verify grasp poses are appropriate for object geometry
2. Adjust gripper approach and grasp parameters
3. Check that object masses and inertias are realistic
4. Implement grasp quality evaluation

### 7. Performance Issues

#### Problem: System running slowly
**Symptoms**:
- Delayed responses to commands
- Low frame rates in simulation
- High CPU/GPU usage

**Solutions**:
1. Optimize simulation settings:
   - Reduce physics update rate if possible
   - Lower rendering quality during development
   - Disable unnecessary visualizations
2. Optimize ROS 2 communication:
   - Use appropriate QoS settings
   - Reduce message frequency for non-critical data
   - Consider using intra-process communication where possible
3. Check for memory leaks in custom nodes
4. Profile nodes to identify bottlenecks

#### Problem: Memory usage growing over time
**Symptoms**:
- System becoming slower over time
- Out of memory errors
- Node crashes due to memory exhaustion

**Solutions**:
1. Implement proper memory management in custom nodes
2. Use generators instead of loading large datasets into memory
3. Regularly call garbage collection in long-running nodes
4. Monitor memory usage with tools like `htop` or `nvidia-smi`

### 8. Integration Issues

#### Problem: Components not working together
**Symptoms**:
- Simulation works in isolation but not with ROS 2
- Perception data not connecting to navigation
- Different parts of the system have inconsistent states

**Solutions**:
1. Verify all components are using the same coordinate frames
2. Check that timing is synchronized (especially with `use_sim_time`)
3. Ensure proper TF tree connections between all components
4. Use the same message types across all components
5. Implement proper state management between components

## Debugging Tools and Commands

### ROS 2 Debugging
```bash
# Check node connections
ros2 node list
ros2 node info <node_name>

# Check topic connections
ros2 topic list
ros2 topic info <topic_name>
ros2 topic echo <topic_name>

# Check service availability
ros2 service list
ros2 service call <service_name> <service_type> <args>

# Check action availability
ros2 action list
ros2 action info <action_name>
```

### Gazebo Debugging
```bash
# Check Gazebo topics
gz topic -l
gz topic -i -t <topic_name>
gz topic -e -t <topic_name>

# Check Gazebo services
gz service -l
```

### System Resource Monitoring
```bash
# Monitor CPU and memory
htop

# Monitor GPU usage
nvidia-smi

# Monitor network usage
netstat -i
iftop  # if installed
```

## Common Error Messages and Solutions

### "No executable found"
- **Cause**: Package not built or not in PATH
- **Solution**: Run `colcon build` and source the setup file

### "TF not found"
- **Cause**: Transform not being published or frames mismatch
- **Solution**: Check TF tree with `ros2 run tf2_tools view_frames`

### "Connection refused"
- **Cause**: Service/node not running or network issue
- **Solution**: Verify that the target service/node is running

### "Segmentation fault"
- **Cause**: Memory access violation, often in C++ nodes
- **Solution**: Check for null pointers, memory leaks, or incorrect library linking

## Prevention Strategies

### Regular Maintenance
- Update packages regularly
- Clean build directories periodically
- Monitor system resources
- Backup important configurations

### Development Best Practices
- Test components in isolation before integration
- Use appropriate logging levels
- Implement error handling
- Follow ROS 2 best practices
- Use simulation for initial testing before hardware deployment

## Getting Help

### When to Seek Help
- Errors persist after consulting this guide
- Issues related to hardware failures
- Complex integration problems
- Performance issues that impact functionality

### Resources
- Official ROS 2 documentation
- Isaac Sim and Isaac ROS documentation
- Gazebo documentation
- GitHub issues for relevant packages
- Robotics Stack Exchange
- NVIDIA Developer Forums

This troubleshooting guide will be updated as new issues are discovered during implementation of the Physical AI & Humanoid Robotics textbook concepts.