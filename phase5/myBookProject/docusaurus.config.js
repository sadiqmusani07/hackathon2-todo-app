// @ts-check
const { themes } = require('prism-react-renderer');

/** @type {import('@docusaurus/types').Config} */
const config = {
  title: 'Physical AI & Humanoid Robotics',
  tagline: 'From Digital Intelligence to Embodied AI',
  favicon: 'img/favicon.ico',

  // Production URL (update if needed)
  url: 'https://your-vercel-domain.vercel.app',
  baseUrl: '/',

  organizationName: 'your-org',
  projectName: 'mybookproject',

  onBrokenLinks: 'warn',
  onBrokenMarkdownLinks: 'warn',

  i18n: {
    defaultLocale: 'en',
    locales: ['en']
  },

  presets: [
    [
      'classic',
      {
        docs: {
          path: 'docs',
          routeBasePath: '/docs',
          sidebarPath: require.resolve('./sidebars.js'),
          editUrl: undefined
        },
        blog: {
          showReadingTime: true
        },
        theme: {
          customCss: require.resolve('./src/css/custom.css')
        }
      }
    ]
  ],

  themes: ['@docusaurus/theme-mermaid'],

  themeConfig: {
    navbar: {
      title: 'Physical AI & Humanoid Robotics',
      items: [
        {
          type: 'doc',
          docId: 'intro',
          position: 'left',
          label: 'Book'
        },
        { to: '/blog', label: 'Blog', position: 'left' }
      ]
    },

    footer: {
      style: 'dark',
      links: [
        {
          title: 'Docs',
          items: [{ label: 'Introduction', to: '/docs/intro' }]
        }
      ],
      copyright: `Copyright © ${new Date().getFullYear()}`
    },

    prism: {
      theme: themes.github,
      darkTheme: themes.dracula
    }
  }
};

module.exports = config;
