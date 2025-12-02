# Docusaurus Setup Workflow

**Version**: 1.0.0
**Last Updated**: 2025-11-29
**Purpose**: Initialize and configure Docusaurus for the book project.

## Overview

This workflow guides the setup of a Docusaurus project to host the generated book content.

## Prerequisites

- Node.js installed (v18+)
- Generated book content in `docs/` (or ready to be generated)

## Steps

### 1. Initialize Docusaurus

If the project is not yet initialized:

```bash
npx create-docusaurus@latest . classic
```

*Note: This assumes you are in the project root. If the directory is not empty, you might need to create it in a subdirectory like `website` and then move files or configure the build.*

### 2. Configure `docusaurus.config.js`

Update the config file:
- Set `title` to your book title.
- Set `url` and `baseUrl`.
- Configure the `docs` plugin to point to your content directory if not using the default `docs/`.

### 3. Integrate Generated Content

The `writer-agent` and `book-editor` are configured to output content to the `docs/` directory.

- Ensure `docs/` contains your markdown files.
- Verify that each file has the correct frontmatter:
  ```markdown
  ---
  title: [Title]
  slug: /[slug]
  sidebar_label: [Label]
  sidebar_position: [N]
  ---
  ```

### 4. Run Local Development Server

```bash
npm start
```

This will start the site at `http://localhost:3000`.

### 5. Build for Production

```bash
npm run build
```

The static files will be generated in `build/`.

## Troubleshooting

- **Missing Frontmatter**: If pages don't appear in the sidebar, check if `sidebar_position` is set.
- **Broken Links**: Use `npm run docusaurus check` to find broken links.
