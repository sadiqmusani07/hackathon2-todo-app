/** @type {import('@docusaurus/plugin-content-docs').SidebarsConfig} */
const sidebars = {
  tutorialSidebar: [
    'intro',
    {
      type: 'category',
      label: 'Module 1: ROS 2 Nervous System',
      items: [
        'module-1/ros2-fundamentals',
        'module-1/python-ai-integration',
        'module-1/urdf-modeling',
      ],
    },
    {
      type: 'category',
      label: 'Module 2: Digital Twin (Gazebo & Unity)',
      items: [
        'module-2/physics-simulation-gazebo',
        'module-2/high-fidelity-rendering-unity',
        'module-2/sensor-environment-integration',
      ],
    },
    {
      type: 'category',
      label: 'Module 3: AI-Robot Brain (NVIDIA Isaac™)',
      items: [
        'module-3/isaac-sim-overview',
        'module-3/isaac-ros-perception-navigation',
        'module-3/nav2-path-planning',
      ],
    },
    {
      type: 'category',
      label: 'Module 4: Vision-Language-Action (VLA)',
      items: [
        'module-4/voice-to-action',
        'module-4/llm-cognitive-planning',
        'module-4/autonomous-humanoid',
      ],
    },
    'glossary',
    'conclusion',
  ],
};

module.exports = sidebars;