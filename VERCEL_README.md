# Deploying Docusaurus to Vercel

This guide explains how to deploy your Docusaurus site to Vercel.

## Prerequisites

- A GitHub repository containing your Docusaurus project
- A Vercel account (https://vercel.com)
- The Vercel CLI installed (optional): `npm install -g vercel`

## Deployment Methods

### Method 1: Connect GitHub Repository (Recommended)

1. **Go to Vercel Dashboard**: https://vercel.com/dashboard
2. **Click "Add New Project"**
3. **Connect your GitHub repository** that contains the Docusaurus site
4. **Vercel will automatically detect** it's a Docusaurus project
5. **Configure the project**:
   - Build Command: `npm run build:vercel`
   - Output Directory: `build`
   - Root Directory: `.` (root)
6. **Click "Deploy"**

### Method 2: Using Vercel CLI

```bash
# Install Vercel CLI
npm install -g vercel

# Login to your account
vercel login

# Deploy the project
vercel --build-env NODE_VERSION=18
```

## Configuration Details

### Build Process

The deployment process uses these scripts:

- `npm run build:vercel` - Prepares the Docusaurus config for Vercel deployment (changes base URL from `/native-book/` to `/`)
- `npm run restore:config` - Restores the original config after deployment

### Base URL Configuration

For Vercel deployment, the site is automatically configured to serve from the root path (`/`) instead of `/native-book/` to work properly with Vercel's domain structure.

### CORS Headers

The vercel.json file includes proper CORS headers to ensure the chatbot integration works correctly.

## Post-Deployment

After deployment to Vercel:

1. The site will be available at: `https://your-project-name.vercel.app`
2. If you used a custom domain, it will be available at your domain
3. The chatbot integration will continue to work with your Hugging Face backend

## Restoring Local Configuration

If you're working locally after a Vercel deployment, run:
```bash
npm run restore:config
```

This will restore the original GitHub Pages configuration.

## Troubleshooting

### Site Not Loading
- Make sure the base URL is correctly configured for the deployment target
- Check that the build completed successfully
- Verify that all static assets are accessible

### Build Errors
- Ensure Node.js version 18+ is used
- Check that all dependencies are properly listed in `package.json`
- Verify that the build command completes successfully locally with `npm run build:vercel`

### Chatbot Not Working
- The chatbot should automatically connect to your Hugging Face backend
- Check browser console for CORS errors
- Verify your Hugging Face backend is running and accessible

### Images or Assets Not Loading
- Ensure all static assets are in the `static/` directory
- Check that paths in your markdown files are relative or properly absolute