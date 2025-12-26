#!/usr/bin/env node
const fs = require('fs');
const path = require('path');

// Script to prepare Docusaurus for Vercel deployment
console.log('Preparing Docusaurus for Vercel deployment...');

// Read the current config
const configPath = path.join(__dirname, 'docusaurus.config.js');
let configContent = fs.readFileSync(configPath, 'utf8');

// Create a temporary config for Vercel (serving from root)
const vercelConfig = configContent
  // Change baseUrl from '/native-book/' to '/'
  .replace("baseUrl: '/native-book/',", "baseUrl: '/',")
  // Update chatbot paths to remove /native-book/ prefix
  .replace("src: '/native-book/js/chatbot.js',", "src: '/js/chatbot.js',")
  .replace("'/native-book/css/chatbot.css',", "'/css/chatbot.css',")
  // Update URL for Vercel deployment
  .replace("url: 'https://sirfanzaidi.github.io',", "url: 'https://your-vercel-domain.vercel.app',");

// Write temporary config
const tempConfigPath = path.join(__dirname, 'docusaurus.config.vercel.js');
fs.writeFileSync(tempConfigPath, vercelConfig);

console.log('Vercel config created at docusaurus.config.vercel.js');

// For the actual Vercel build, we'll temporarily replace the config
const originalConfigPath = path.join(__dirname, 'docusaurus.config.original.js');

// Backup original config
fs.writeFileSync(originalConfigPath, configContent);

// Use Vercel config temporarily
fs.writeFileSync(configPath, vercelConfig);

console.log('Switched to Vercel config. Run `npm run build` now.');
console.log('After deployment, run `restore-config.js` to revert.');