// @ts-check
// Docusaurus configuration for Physical AI & Humanoid Robotics Book

/** @type {import('@docusaurus/types').Config} */
const config = {
  title: 'Physical AI & Humanoid Robotics',
  tagline: 'A Spec-Driven Guide to ROS 2, Simulation, Perception, and Voice Control',
  favicon: 'img/logo.svg',

  // Set the production url of your site here
  url: 'https://sirfanzaidi.github.io',
  // Set the /<baseUrl>/ pathname under which your site is served
  baseUrl: '/native-book/',

  // GitHub pages deployment config
  organizationName: 'sirfanzaidi',
  projectName: 'native-book',
  deploymentBranch: 'gh-pages',

  onBrokenLinks: 'warn',
  onBrokenMarkdownLinks: 'warn',
  i18n: {
    defaultLocale: 'en',
    locales: ['en'],
  },

  presets: [
    [
      'classic',
      /** @type {import('@docusaurus/preset-classic').Options} */
      ({
        docs: {
          sidebarPath: require.resolve('./sidebars.js'),
          routeBasePath: '/',
          editUrl:
            'https://github.com/native-book/native-book/tree/main/',
        },
        blog: false,
        theme: {
          customCss: require.resolve('./src/css/custom.css'),
        },
      }),
    ],
  ],

  // RAG Chatbot Integration
  scripts: [
    {
      src: '/native-book/js/chatbot.js',
      async: true,
    },
  ],
  stylesheets: [
    '/native-book/css/chatbot.css',
  ],

  themeConfig:
    /** @type {import('@docusaurus/preset-classic').ThemeConfig} */
    ({
      colorMode: {
        defaultMode: 'light',
        disableSwitch: false,
        respectPrefersColorScheme: true,
      },
      image: 'img/docusaurus-social-card.jpg',
      navbar: {
        title: 'Physical AI Book',
        logo: {
          alt: 'Book Logo',
          src: 'img/logo.svg',
        },
        items: [
          {
            type: 'docSidebar',
            sidebarId: 'tutorialSidebar',
            position: 'left',
            label: 'Chapters',
          },
          {
            href: '/',
            label: 'Explore Modules',
            position: 'left',
          },
          {
            href: 'https://github.com/native-book/native-book',
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
                label: 'Module 1: ROS 2',
                to: '/01-module-1-ros2/01-introduction',
              },
            ],
          },
          {
            title: 'Community',
            items: [
              {
                label: 'GitHub',
                href: 'https://github.com/native-book/native-book',
              },
            ],
          },
        ],
        copyright: `Copyright © ${new Date().getFullYear()} Physical AI & Humanoid Robotics Book. Built with Docusaurus.`,
      },
    }),

  plugins: [],
};

module.exports = config;
