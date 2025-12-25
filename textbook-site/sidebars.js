// @ts-check

/** @type {import('@docusaurus/plugin-content-docs').SidebarsConfig} */
const sidebars = {
  tutorialSidebar: [
    {
      type: 'category',
      label: 'Introduction',
      items: ['intro'],
      collapsed: false,
    },
    {
      type: 'category',
      label: 'Module 1: The Robotic Nervous System (ROS 2)',
      items: [
        'module-1/index',
        'module-1/ros2-introduction',
        'module-1/ros2-nodes-topics-services',
        'module-1/rclpy-python-integration',
        'module-1/urdf-robot-description',
        'module-1/ros2-best-practices',
        'module-1/exercises',
      ],
      collapsed: false,
    },
    {
      type: 'category',
      label: 'Module 2: The Digital Twin (Gazebo & Unity)',
      items: [
        'module-2/index',
        'module-2/gazebo-introduction',
        'module-2/physics-simulation',
        'module-2/sensor-simulation',
        'module-2/ros2-gazebo-integration',
        'module-2/gazebo-bridge',
        'module-2/exercises',
      ],
      collapsed: false,
    },
    {
      type: 'category',
      label: 'Module 3: The AI-Robot Brain (NVIDIA Isaac™)',
      items: [
        'module-3/index',
        'module-3/isaac-sim-introduction',
        'module-3/synthetic-data-generation',
        'module-3/isaac-ros-integration',
        'module-3/nav2-path-planning',
        'module-3/ai-perception-systems',
        'module-3/exercises',
      ],
      collapsed: false,
    },
    {
      type: 'category',
      label: 'Module 4: Vision-Language-Action (VLA)',
      items: [
        'module-4/index',
        'module-4/voice-processing',
        'module-4/cognitive-planning',
        'module-4/language-action-mapping',
        'module-4/computer-vision-object-id',
        'module-4/manipulation-examples',
        'module-4/exercises',
      ],
      collapsed: false,
    },
    {
      type: 'category',
      label: 'Capstone: Autonomous Humanoid Project',
      items: [
        'capstone/index',
        'capstone/implementation',
        'capstone/testing-validation',
        'capstone/exercises',
      ],
      collapsed: false,
    },
    {
      type: 'category',
      label: 'Appendices',
      items: [
        'appendices/hardware-setup',
        'appendices/troubleshooting',
        'appendices/glossary',
        'appendices/references',
      ],
      collapsed: true,
    },
  ],
};

module.exports = sidebars;