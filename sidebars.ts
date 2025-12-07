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
      label: 'Chapter 1 – Quarter Overview & Physical AI',
      items: [
        'chapter-1-lesson-1',
        'chapter-1-lesson-2',
        'chapter-1-lesson-3',
      ],
    },
    {
      type: 'category',
      label: 'Chapter 2 – ROS 2: Robotic Nervous System',
      items: [
        'chapter-2-lesson-1',
        'chapter-2-lesson-2',
        'chapter-2-lesson-3',
      ],
    },
    {
      type: 'category',
      label: 'Chapter 3 – Digital Twin (Gazebo & Unity)',
      items: [
        'chapter-3-lesson-1',
        'chapter-3-lesson-2',
        'chapter-3-lesson-3',
      ],
    },
    {
      type: 'category',
      label: 'Chapter 4 – NVIDIA Isaac: AI-Robot Brain',
      items: [
        'chapter-4-lesson-1',
        'chapter-4-lesson-2',
        'chapter-4-lesson-3',
      ],
    },
    {
      type: 'category',
      label: 'Chapter 5 – Vision-Language-Action (VLA)',
      items: [
        'chapter-5-lesson-1',
        'chapter-5-lesson-2',
        'chapter-5-lesson-3',
      ],
    },
    {
      type: 'category',
      label: 'Chapter 6 – Why Physical AI Matters',
      items: [
        'chapter-6-lesson-1',
        'chapter-6-lesson-2',
        'chapter-6-lesson-3',
      ],
    },
    {
      type: 'category',
      label: 'Chapter 7 – Learning Outcomes & Capstone',
      items: [
        'chapter-7-lesson-1',
        'chapter-7-lesson-2',
        'chapter-7-lesson-3',
      ],
    },
    // Keep existing docs as reference
    'quarter-overview',
    'module-1-ros2',
    'module-2-digital-twin',
    'module-3-nvidia-isaac',
    'module-4-vla',
    'why-physical-ai-matters',
    'learning-outcomes',
  ],
};

export default sidebars;
