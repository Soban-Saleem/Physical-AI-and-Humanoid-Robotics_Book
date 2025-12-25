# Path Planning with Nav2 for Humanoid Robotics

Navigation is a critical capability for humanoid robots, enabling them to move through complex environments safely and efficiently. The Navigation2 (Nav2) framework provides a comprehensive path planning and navigation solution that can be adapted for bipedal humanoid robots with unique locomotion characteristics.

## Learning Objectives

By the end of this section, you will be able to:
- Configure Nav2 for humanoid robot navigation in complex environments
- Implement custom path planners suitable for bipedal locomotion
- Integrate Nav2 with Isaac Sim for simulation-based development
- Customize Nav2 costmaps for humanoid-specific navigation challenges
- Implement recovery behaviors appropriate for humanoid robots
- Validate navigation performance in simulation before real-world deployment
- Design navigation behaviors that account for humanoid robot dynamics

## Introduction to Nav2

Navigation2 (Nav2) is the navigation stack for ROS 2, designed to provide path planning, navigation, and obstacle avoidance capabilities. For humanoid robots, Nav2 needs to be adapted to account for:

- **Bipedal Dynamics**: Different locomotion patterns compared to wheeled robots
- **Stability Requirements**: Need to maintain balance during navigation
- **Footstep Planning**: Consideration of where to place feet for stable walking
- **Human-Scale Environments**: Navigation in spaces designed for human-sized robots

### Key Nav2 Components

1. **Global Planner**: Computes optimal path from start to goal
2. **Local Planner**: Executes path while avoiding obstacles in real-time
3. **Costmap 2D**: Maintains obstacle information in a 2D grid
4. **Behavior Tree**: Orchestrates navigation tasks and recovery behaviors
5. **Recovery Behaviors**: Handle navigation failures and obstacles

## Nav2 Architecture for Humanoid Robots

### Standard Nav2 Architecture

```
[Map Server] → [Global Costmap] → [Global Planner] → [Behavior Tree] → [Local Planner] → [Controller]
                                           ↓
                                   [Local Costmap] ← [Sensor Fusion]
```

### Humanoid-Specific Modifications

For humanoid robots, we need to consider:

1. **Footstep Planner**: Additional layer for planning where to place feet
2. **Stability Checker**: Ensures planned paths maintain robot balance
3. **Bipedal Controller**: Specialized controller for legged locomotion
4. **Dynamic Window**: Adapted for bipedal dynamics instead of differential drive

### Nav2 Configuration for Humanoid Robots

```yaml
# config/nav2_params_humanoid.yaml
amcl:
  ros__parameters:
    use_sim_time: True
    alpha1: 0.2
    alpha2: 0.2
    alpha3: 0.2
    alpha4: 0.2
    alpha_slowweight: 0.0
    base_frame_id: "base_link"
    beam_skip_distance: 0.5
    beam_skip_error_threshold: 0.9
    beam_skip_threshold: 0.3
    do_beamskip: False
    global_frame_id: "map"
    lambda_short: 0.1
    laser_likelihood_max_dist: 2.0
    laser_max_range: 100.0
    laser_min_range: -1.0
    laser_model_type: "likelihood_field"
    max_beams: 60
    max_particles: 2000
    min_particles: 500
    odom_frame_id: "odom"
    pf_err: 0.05
    pf_z: 0.99
    recovery_alpha_fast: 0.0
    recovery_alpha_slow: 0.0
    resample_interval: 1
    robot_model_type: "nav2_amcl::DifferentialMotionModel"
    save_pose_rate: 0.5
    sigma_hit: 0.2
    tf_broadcast: True
    transform_tolerance: 1.0
    update_min_a: 0.2
    update_min_d: 0.25
    z_hit: 0.5
    z_max: 0.05
    z_rand: 0.5
    z_short: 0.05

bt_navigator:
  ros__parameters:
    use_sim_time: True
    global_frame: "map"
    robot_base_frame: "base_link"
    odom_topic: "odom"
    bt_loop_duration: 10
    default_server_timeout: 20
    enable_groot_monitoring: True
    groot_zmq_publisher_port: 1666
    groot_zmq_server_port: 1667
    # Humanoid-specific behavior tree
    default_nav_to_pose_bt_xml: "humanoid_nav_to_pose_w_replanning_and_recovery.xml"
    default_nav_through_poses_bt_xml: "humanoid_nav_through_poses_w_replanning_and_recovery.xml"

controller_server:
  ros__parameters:
    use_sim_time: True
    controller_frequency: 20.0
    min_x_velocity_threshold: 0.001
    min_y_velocity_threshold: 0.5
    min_theta_velocity_threshold: 0.001
    # Humanoid-specific controller
    progress_checker_plugin: "progress_checker"
    goal_checker_plugin: "goal_checker"
    controller_plugins: ["FollowPath"]

    # Humanoid-specific FollowPath controller
    FollowPath:
      plugin: "nav2_mppi_controller::MPPICtrl"
      # MPPICtrl is more suitable for humanoid robots than pure pursuit
      sim_time: 1.7
      vx_samples: 20
      vy_samples: 5
      w_samples: 20
      trajectory_generator_name: "dwb_core::HumanoidTrajectoryGenerator"
      # Humanoid-specific trajectory generator
      progress_checker_name: "progress_checker"
      goal_checker_name: "goal_checker"

local_costmap:
  local_costmap:
    ros__parameters:
      update_frequency: 5.0
      publish_frequency: 2.0
      global_frame: "odom"
      robot_base_frame: "base_link"
      use_sim_time: True
      rolling_window: true
      width: 6
      height: 6
      resolution: 0.05  # Higher resolution for precise footstep planning
      robot_radius: 0.3  # Adjust for humanoid robot dimensions
      plugins: ["voxel_layer", "inflation_layer"]
      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        cost_scaling_factor: 3.0
        inflation_radius: 0.55
      voxel_layer:
        plugin: "nav2_costmap_2d::VoxelLayer"
        enabled: True
        publish_voxel_map: True
        origin_z: 0.0
        z_resolution: 0.2
        z_voxels: 10
        max_obstacle_height: 2.0
        mark_threshold: 0
        observation_sources: scan
        scan:
          topic: /scan
          max_obstacle_height: 2.0
          clearing: True
          marking: True
          data_type: "LaserScan"
          raytrace_max_range: 3.0
          raytrace_min_range: 0.0
          obstacle_max_range: 2.5
          obstacle_min_range: 0.0

global_costmap:
  global_costmap:
    ros__parameters:
      update_frequency: 1.0
      publish_frequency: 0.5
      global_frame: "map"
      robot_base_frame: "base_link"
      use_sim_time: True
      robot_radius: 0.3  # Adjust for humanoid robot
      resolution: 0.05   # Higher resolution for detailed planning
      track_unknown_space: true
      plugins: ["static_layer", "obstacle_layer", "inflation_layer"]
      obstacle_layer:
        plugin: "nav2_costmap_2d::ObstacleLayer"
        enabled: True
        observation_sources: scan
        scan:
          topic: /scan
          max_obstacle_height: 2.0
          clearing: True
          marking: True
          data_type: "LaserScan"
          raytrace_max_range: 3.0
          raytrace_min_range: 0.0
          obstacle_max_range: 2.5
          obstacle_min_range: 0.0
      static_layer:
        plugin: "nav2_costmap_2d::StaticLayer"
        map_subscribe_transient_local: True
      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        cost_scaling_factor: 3.0
        inflation_radius: 0.55

planner_server:
  ros__parameters:
    expected_planner_frequency: 20.0
    use_sim_time: True
    planner_plugins: ["GridBased"]
    GridBased:
      # For humanoid robots, we might want to use a custom planner
      # that considers footstep placement
      plugin: "nav2_navfn_planner::NavfnPlanner"
      tolerance: 0.5
      use_astar: false
      allow_unknown: true