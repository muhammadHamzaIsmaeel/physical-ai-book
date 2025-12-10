/**
 * Creating a sidebar enables you to:
 - create an ordered group of docs
 - render a sidebar for each doc of that group
 - provide next/previous navigation

 The sidebars can be generated from the filesystem, or explicitly defined here.

 Create as many sidebars as you want.
 */

// @ts-check

/** @type {import('@docusaurus/plugin-content-docs').SidebarsConfig} */
const sidebars = {
  // Main tutorial sidebar for the book
  tutorialSidebar: [
    {
      type: 'doc',
      id: 'intro',
      label: 'Introduction',
    },
    {
      type: 'category',
      label: 'Part I: Foundation',
      collapsed: false,
      items: [
        {
          type: 'category',
          label: '01. Why Physical AI Is the Next Frontier',
          items: [
            {
              type: 'doc',
              id: 'why-physical-ai/index',
              label: 'Overview',
            },
            {
              type: 'doc',
              id: 'why-physical-ai/why-physical-ai-quiz',
              label: 'Quiz',
            },
          ],
        },
        {
          type: 'category',
          label: '02. The Hardware You Actually Need in 2026',
          items: [
            {
              type: 'doc',
              id: 'hardware-2026/index',
              label: 'Overview',
            },
            {
              type: 'doc',
              id: 'hardware-2026/hardware-2026-quiz',
              label: 'Quiz',
            },
          ],
        },
      ],
    },
    {
      type: 'category',
      label: 'Part II: ROS 2 & Simulation',
      collapsed: false,
      items: [
        {
          type: 'category',
          label: '03. ROS 2 – The Robotic Nervous System',
          items: [
            {
              type: 'doc',
              id: 'ros2-fundamentals/index',
              label: 'Overview',
            },
            {
              type: 'doc',
              id: 'ros2-fundamentals/nodes-topics',
              label: 'Nodes & Topics',
            },
            {
              type: 'doc',
              id: 'ros2-fundamentals/services-actions',
              label: 'Services & Actions',
            },
            {
              type: 'doc',
              id: 'ros2-fundamentals/parameters-launch',
              label: 'Parameters & Launch Files',
            },
            {
              type: 'doc',
              id: 'ros2-fundamentals/debugging-tools',
              label: 'Debugging Tools',
            },
            {
              type: 'doc',
              id: 'ros2-fundamentals/quiz',
              label: 'Quiz',
            },
          ],
        },
        {
          type: 'category',
          label: '04. URDF & Digital Twins',
          items: [
            {
              type: 'doc',
              id: 'urdf-digital-twins/index',
              label: 'Overview',
            },
            {
              type: 'doc',
              id: 'urdf-digital-twins/quiz',
              label: 'Quiz',
            },
          ],
        },
        {
          type: 'category',
          label: '05. Gazebo vs Isaac Sim',
          items: [
            {
              type: 'doc',
              id: 'simulation-ecosystem/index',
              label: 'Overview',
            },
            {
              type: 'doc',
              id: 'simulation-ecosystem/quiz',
              label: 'Quiz',
            },
          ],
        },
        {
          type: 'category',
          label: '06. NVIDIA Isaac Platform',
          items: [
            {
              type: 'doc',
              id: 'isaac-platform/index',
              label: 'Overview',
            },
            {
              type: 'doc',
              id: 'isaac-platform/quiz',
              label: 'Quiz',
            },
          ],
        },
      ],
    },
    {
      type: 'category',
      label: 'Part III: Core Robotics',
      collapsed: false,
      items: [
        {
          type: 'category',
          label: '07. Perception Stack',
          items: [
            {
              type: 'doc',
              id: 'perception-stack/perception-stack',
              label: 'Overview',
            },
            {
              type: 'doc',
              id: 'perception-stack/quiz',
              label: 'Quiz',
            },
          ],
        },
        {
          type: 'category',
          label: '08. Bipedal Locomotion',
          items: [
            {
              type: 'doc',
              id: 'bipedal-locomotion/index',
              label: 'Overview',
            },
            {
              type: 'doc',
              id: 'bipedal-locomotion/quiz',
              label: 'Quiz',
            },
          ],
        },
        {
          type: 'category',
          label: '09. Dexterous Manipulation',
          items: [
            {
              type: 'doc',
              id: 'dexterous-manipulation/index',
              label: 'Overview',
            },
            {
              type: 'doc',
              id: 'dexterous-manipulation/quiz',
              label: 'Quiz',
            },
          ],
        },
      ],
    },
    {
      type: 'category',
      label: 'Part IV: Intelligence Layer',
      collapsed: false,
      items: [
        {
          type: 'category',
          label: '10. Vision-Language-Action Models',
          items: [
            {
              type: 'doc',
              id: 'vla-models/index',
              label: 'Overview',
            },
            {
              type: 'doc',
              id: 'vla-models/quiz',
              label: 'Quiz',
            },
          ],
        },
        {
          type: 'category',
          label: '11. Voice-to-Action Pipeline',
          items: [
            {
              type: 'doc',
              id: 'voice-to-action/index',
              label: 'Overview',
            },
            {
              type: 'doc',
              id: 'voice-to-action/quiz',
              label: 'Quiz',
            },
          ],
        },
        {
          type: 'doc',
          id: 'sim-to-real/sim-to-real',
          label: '12. Sim-to-Real Transfer',
        },
      ],
    },
    {
      type: 'category',
      label: 'Part V: Integration',
      collapsed: false,
      items: [
        {
          type: 'doc',
          id: 'capstone-butler/capstone-butler',
          label: '13. Capstone: Autonomous Butler',
        },
      ],
    },
    {
      type: 'category',
      label: 'Appendices',
      collapsed: true,
      items: [
        {
          type: 'category',
          label: 'A. Lab Build Guides',
          link: {type: 'doc', id: 'appendices/lab-build-guides'},
          items: [
            'appendices/lab-build-guides/economy-tier',
            'appendices/lab-build-guides/mid-tier',
            'appendices/lab-build-guides/premium-tier',
          ],
        },
        {
          type: 'doc',
          id: 'appendices/troubleshooting-bible',
          label: 'B. Troubleshooting Bible',
        },
        {
          type: 'doc',
          id: 'appendices/future-roadmap',
          label: 'C. Future Roadmap (2026-2030)',
        },
      ],
    },
  ],
};

module.exports = sidebars;
