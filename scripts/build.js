#!/usr/bin/env node
// Build script that conditionally adds --localstorage-file flag for Node.js 25+
const { execSync } = require('child_process');
const path = require('path');
const os = require('os');

// Get Node.js version
const nodeVersion = process.version.match(/^v(\d+)/)[1];
const nodeVersionNum = parseInt(nodeVersion, 10);

// Get the project root directory
const projectRoot = path.resolve(__dirname, '..');
const docusaurusBin = path.join(projectRoot, 'node_modules', '.bin', 'docusaurus');

// Build command - use npx for cross-platform compatibility
let buildCommand = `npx docusaurus build`;

if (nodeVersionNum >= 25) {
  // Node.js 25+ requires --localstorage-file flag
  // Use a temp file path that works on both Unix and Windows
  const tempDir = os.tmpdir();
  const localStorageFile = path.join(tempDir, 'docusaurus-localstorage');
  buildCommand = `node --localstorage-file="${localStorageFile}" "${docusaurusBin}" build`;
}

console.log(`Building with Node.js ${nodeVersion}...`);
try {
  execSync(buildCommand, { stdio: 'inherit', cwd: projectRoot });
} catch (error) {
  process.exit(error.status || 1);
}

