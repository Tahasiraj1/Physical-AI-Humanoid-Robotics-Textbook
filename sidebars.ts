import type {SidebarsConfig} from '@docusaurus/plugin-content-docs';

const sidebars: SidebarsConfig = {
  textbookSidebar: [
    'intro',
    {
      type: 'category',
      label: 'Modules',
      items: [
        {
          type: 'category',
          label: 'Module 1: The Robotic Nervous System (ROS 2)',
          items: [
            'modules/module-1-ros2-nervous-system/module-1-ros2-nervous-system',
            'modules/module-1-ros2-nervous-system/introduction',
            'modules/module-1-ros2-nervous-system/ros2-fundamentals',
            'modules/module-1-ros2-nervous-system/communication-patterns',
            'modules/module-1-ros2-nervous-system/humanoid-applications',
            'modules/module-1-ros2-nervous-system/workspace-overview',
            'modules/module-1-ros2-nervous-system/glossary',
          ],
        },
      ],
    },
  ],
};

export default sidebars;

