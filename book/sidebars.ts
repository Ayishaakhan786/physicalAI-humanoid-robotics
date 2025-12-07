import type {SidebarsConfig} from '@docusaurus/plugin-content-docs';

// This runs in Node.js - Don't use client-side code here (browser APIs, JSX...)

/**
 * Creating a sidebar enables you to:
 - create an ordered group of docs
 - render a sidebar for each doc of that group
 - provide next/previous navigation

 The sidebars can be generated from the filesystem, or explicitly defined here.

 Create as many sidebars as you want.
 */
const sidebars: SidebarsConfig = {
  tutorialSidebar: [
    {
      type: 'category',
      label: 'Preface',
      link: {type: 'doc', id: 'preface'},
      items: [],
    },
    {
      type: 'category',
      label: 'Module 1: The Robotic Nervous System (ROS 2)',
      link: {type: 'doc', id: 'module-01-ros2/intro'},
      items: [
        'module-01-ros2/intro',
        'module-01-ros2/ros-architecture',
        'module-01-ros2/nodes-topics',
        'module-01-ros2/services-actions',
        'module-01-ros2/robot-control',
      ],
    },
    {
      type: 'category',
      label: 'Module 2: The Digital Twin (Gazebo & Unity)',
      link: {type: 'doc', id: 'module-02-digital-twin/intro'},
      items: [
        'module-02-digital-twin/intro',
        'module-02-digital-twin/gazebo-sim',
        'module-02-digital-twin/urdf-models',
        'module-02-digital-twin/advanced-gazebo',
        'module-02-digital-twin/unity-robotics',
      ],
    },
    {
      type: 'category',
      label: 'Module 3: The AI-Robot Brain (NVIDIA Isaac)',
      link: {type: 'doc', id: 'module-03-isaac/intro'},
      items: [
        'module-03-isaac/intro',
        'module-03-isaac/isaac-sim-basics',
        'module-03-isaac/robotics-isaac',
        'module-03-isaac/perception-ai',
        'module-03-isaac/advanced-isaac',
      ],
    },
    {
      type: 'category',
      label: 'Module 4: Vision-Language-Action (VLA)',
      link: {type: 'doc', id: 'module-04-vla/intro'},
      items: [
        'module-04-vla/intro',
        'module-04-vla/speech-recognition',
        'module-04-vla/nlu',
        'module-04-vla/action-planning',
        'module-04-vla/vla-capstone',
      ],
    },
    {
      type: 'category',
      label: 'Appendices',
      items: [
        'hardware-appendix',
        'capstone-appendix',
      ],
    },
    {
      type: 'category',
      label: 'References & Glossary',
      items: [
        'references',
        'glossary',
      ],
    },
  ],
};

export default sidebars;
