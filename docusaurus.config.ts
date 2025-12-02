import { themes as prismThemes } from "prism-react-renderer";
import type { Config } from "@docusaurus/types";
import type * as Preset from "@docusaurus/preset-classic";

const config: Config = {
  title: "AI Native Software Development",
  tagline: "Colearning Agentic AI with Python and TypeScript – Spec Driven Reusable Intelligence",
  favicon: "img/favicon.ico",

  future: {
    v4: true,
  },

  url: "https://ai-native.panaversity.org",
  baseUrl: "/",

  organizationName: "panaversity",
  projectName: "ai-native-software-development",
  trailingSlash: false,

  onBrokenLinks: "warn",

  i18n: {
    defaultLocale: "en",
    locales: ["en"],
  },

  plugins: [
    './plugins/docusaurus-webpack-alias',
  ],

  presets: [
    [
      "classic",
      {
        docs: {
          path: "docs",
          sidebarPath: "./sidebars.ts",
        },
        blog: false,
        theme: {
          customCss: "./src/css/custom.css",
        },
      } satisfies Preset.Options,
    ],
  ],

  themeConfig: {
    image: "img/book-cover-page.png",
    colorMode: {
      defaultMode: "dark",
      disableSwitch: true,
      respectPrefersColorScheme: false,
    },
    docs: {
      sidebar: {
        hideable: true,
      },
    },
    navbar: {
      title: "AI Native Development",
      hideOnScroll: false,
      items: [
        {
          href: "https://github.com/panaversity/ai-native-software-development",
          label: "GitHub",
          position: "right",
        },
      ],
    },
    footer: {
      style: "dark",
      links: [
        {
          title: "Learn",
          items: [
            {
              label: "Start Your Journey",
              to: "/docs/preface-agent-native",
            },
          ],
        },
        {
          title: "Community",
          items: [
            {
              label: "YouTube",
              href: "https://youtube.com/@panaversity",
            },
            {
              label: "LinkedIn",
              href: "https://linkedin.com/company/panaversity",
            },
          ],
        },
        {
          title: "Resources",
          items: [
            {
              label: "GitHub Repository",
              href: "https://github.com/panaversity/ai-native-software-development",
            },
          ],
        },
      ],
      copyright: `Copyright © ${new Date().getFullYear()} <strong>Panaversity</strong> • AI Native Software Development`,
    },
    prism: {
      theme: prismThemes.github,
      darkTheme: prismThemes.dracula,
    },
  } satisfies Preset.ThemeConfig,
};

export default config;
