#!/usr/bin/env node
const fs = require('fs');
const path = require('path');

// Script to restore original Docusaurus config after Vercel deployment
console.log('Restoring original Docusaurus config...');

const originalConfigPath = path.join(__dirname, 'docusaurus.config.original.js');
const currentConfigPath = path.join(__dirname, 'docusaurus.config.js');

// Check if backup exists
if (fs.existsSync(originalConfigPath)) {
  const originalConfig = fs.readFileSync(originalConfigPath, 'utf8');
  fs.writeFileSync(currentConfigPath, originalConfig);
  console.log('Original config restored.');

  // Clean up backup file
  if (fs.existsSync(originalConfigPath)) {
    fs.unlinkSync(originalConfigPath);
    console.log('Backup config removed.');
  }
} else {
  console.log('No backup config found. Nothing to restore.');
}

console.log('Config restoration complete.');