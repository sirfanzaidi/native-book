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
  .replace(/baseUrl: '\/native-book\/',/g, "baseUrl: '/',")
  // Update chatbot paths to remove /native-book/ prefix
  .replace(/src: '\/native-book\/js\/chatbot\.js',/g, "src: '/js/chatbot.js',")
  .replace(/'\/native-book\/css\/chatbot\.css',/g, "'/css/chatbot.css',")
  // Update URL for Vercel deployment
  .replace(/url: 'https:\/\/sirfanzaidi\.github\.io',/g, "url: 'https://your-vercel-domain.vercel.app',");

// For the actual Vercel build, we'll temporarily replace the config
const originalConfigPath = path.join(__dirname, 'docusaurus.config.original.js');

// Backup original config
fs.writeFileSync(originalConfigPath, configContent);

// Use Vercel config temporarily
fs.writeFileSync(configPath, vercelConfig);

console.log('Switched to Vercel config. Ready for build.');
console.log('After deployment, run `npm run restore:config` to revert.');