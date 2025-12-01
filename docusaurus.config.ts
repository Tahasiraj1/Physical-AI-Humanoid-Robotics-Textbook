import {themes as prismThemes} from 'prism-react-renderer';
import type {Config} from '@docusaurus/types';
import type * as Preset from '@docusaurus/preset-classic';

const config: Config = {
  title: 'Physical AI Humanoid Robotics Textbook',
  tagline: 'Comprehensive guide to Physical AI and Humanoid Robotics',
  favicon: 'img/favicon.ico',

  future: {
    v4: true,
  },

  // GitHub Pages configuration
  url: 'https://Tahasiraj1.github.io',
  baseUrl: '/Physical-AI-Humanoid-Robotics-Textbook/',
  trailingSlash: false,

  // GitHub pages deployment config
  organizationName: 'Tahasiraj1',
  projectName: 'Physical-AI-Humanoid-Robotics-Textbook',

  onBrokenLinks: 'warn',

  i18n: {
    defaultLocale: 'en',
    locales: ['en'],
  },

  presets: [
    [
      'classic',
      {
        docs: {
          sidebarPath: './sidebars.ts',
          routeBasePath: '/', // Serve docs at root
          editUrl: undefined, // Remove edit links for now
        },
        blog: false, // Disable blog for textbook
        theme: {
          customCss: './src/css/custom.css',
        },
      } satisfies Preset.Options,
    ],
  ],

  themeConfig: {
    image: 'img/docusaurus-social-card.jpg',
    colorMode: {
      respectPrefersColorScheme: true,
    },
    navbar: {
      title: 'Physical AI Humanoid Robotics',
      logo: {
        alt: 'Physical AI Humanoid Robotics Logo',
        src: 'img/logo.svg',
      },
      items: [
        {
          type: 'docSidebar',
          sidebarId: 'textbookSidebar',
          position: 'left',
          label: 'Textbook',
        },
        {
          href: 'https://github.com/Tahasiraj1/Physical-AI-Humanoid-Robotics-Textbook',
          label: 'GitHub',
          position: 'right',
        },
      ],
    },
    footer: {
      style: 'dark',
      links: [
        {
          title: 'Modules',
          items: [
            {
              label: 'Module 1: ROS 2 Nervous System',
              to: '/modules/module-1-ros2-nervous-system/',
            },
          ],
        },
        {
          title: 'Resources',
          items: [
            {
              label: 'GitHub',
              href: 'https://github.com/Tahasiraj1/Physical-AI-Humanoid-Robotics-Textbook',
            },
          ],
        },
      ],
      copyright: `Copyright © ${new Date().getFullYear()} Physical AI Humanoid Robotics Textbook. Built with Docusaurus.`,
    },
    prism: {
      theme: prismThemes.github,
      darkTheme: prismThemes.dracula,
      additionalLanguages: ['python', 'bash'],
    },
  } satisfies Preset.ThemeConfig,
};

export default config;

