# Deployment Guide: Physical AI & Humanoid Robotics Textbook

## Overview

This guide explains how to deploy the Physical AI & Humanoid Robotics textbook to GitHub Pages with all integrated features (RAG chatbot, authentication, personalization, Urdu translation).

## Prerequisites

Before deploying, ensure you have:

1. **GitHub Repository**: A repository created at `https://github.com/Soban-Saleem/Physical-AI-and-Humanoid-Robotics_Book`
2. **GitHub Pages Settings**: GitHub Pages enabled in the repository settings
3. **Backend Services**: All backend services (API, database, vector storage) deployed and accessible
4. **Environment Variables**: Backend service URLs configured in the Docusaurus config

## Deployment Configuration

### 1. Backend Service URLs

Update the `docusaurus.config.js` file with your deployed backend service URLs:

```javascript
// In docusaurus.config.js
const config = {
  // ... other configuration ...
  
  themeConfig: {
    // ... other theme configuration ...
    
    // Add your backend service URLs here
    backendServices: {
      ragApiUrl: 'https://your-rag-service.com/api',
      authApiUrl: 'https://your-auth-service.com/api',
      personalizationApiUrl: 'https://your-personalization-service.com/api',
      translationApiUrl: 'https://your-translation-service.com/api'
    }
  }
};
```

### 2. GitHub Actions Workflow

The GitHub Actions workflow is already configured in `.github/workflows/deploy.yml`:

```yaml
name: Deploy to GitHub Pages

on:
  push:
    branches: [master]  # Changed from 'main' to 'master' to match your branch name
  # Allows you to run this workflow manually from the Actions tab
  workflow_dispatch:

# Sets permissions of the GITHUB_TOKEN to allow deployment to GitHub Pages
permissions:
  contents: read
  pages: write
  id-token: write

# Allow only one concurrent deployment, skipping runs queued between the run in-progress and latest queued.
# However, do NOT cancel in-progress runs as we want to allow these production deployments to complete.
concurrency:
  group: "pages"
  cancel-in-progress: false

jobs:
  # Build job
  build:
    runs-on: ubuntu-latest
    steps:
      - name: Checkout
        uses: actions/checkout@v4
      - name: Setup Node
        uses: actions/setup-node@v4
        with:
          node-version: 18
          cache: npm
      - name: Install dependencies
        run: |
          cd textbook-site
          npm ci
      - name: Build website
        run: |
          cd textbook-site
          npm run build
      # Deploy to GitHub Pages
      - name: Deploy to GitHub Pages
        uses: peaceiris/actions-gh-pages@v3
        with:
          github_token: ${{ secrets.GITHUB_TOKEN }}
          # Build output to publish to the `gh-pages` branch:
          publish_dir: ./textbook-site/build
          # The following lines assign commit authorship to the official
          # GH-Actions bot for deploys to `gh-pages` branch:
          publish_branch: gh-pages
          force_orphan: true
```

## Deployment Steps

### 1. Prepare the Repository

1. Ensure your GitHub repository is properly set up at `https://github.com/Soban-Saleem/Physical-AI-and-Humanoid-Robotics_Book`
2. In the repository settings, go to "Pages" section
3. Set the source to "Deploy from a branch"
4. Select "gh-pages" as the branch and "/" as the folder
5. Click "Save"

### 2. Configure Backend Services

Before deploying, you must have your backend services deployed and accessible:

1. **RAG Service**: Deploy your RAG chatbot service to a cloud provider
2. **Authentication Service**: Deploy your authentication service with Better-Auth
3. **Personalization Service**: Deploy your personalization service
4. **Translation Service**: Deploy your Urdu translation service

### 3. Update Configuration

Update the `docusaurus.config.js` file with your backend service URLs before pushing.

### 4. Commit and Push Changes

```bash
# Commit the changes
git add .
git commit -m "feat: update configuration for deployment"

# Push to trigger GitHub Actions
git push origin master
```

### 5. Monitor GitHub Actions

1. Go to your GitHub repository
2. Click on the "Actions" tab
3. Monitor the "Deploy to GitHub Pages" workflow
4. Wait for it to complete successfully

### 6. Verify Deployment

1. Visit your deployed site at: `https://soban-saleem.github.io/Physical-AI-and-Humanoid-Robotics_Book/`
2. Verify that all textbook content is accessible
3. Test the RAG chatbot functionality
4. Verify authentication and personalization features work
5. Test Urdu translation functionality per chapter

## Backend Services Architecture

For the integrated features to work, you need to deploy the following backend services separately:

### 1. RAG Service (Qdrant + FastAPI)
- Vector database (Qdrant Cloud) for storing textbook content embeddings
- FastAPI service for processing queries and retrieving relevant content
- Integration with OpenAI APIs for response generation

### 2. Authentication Service (Better-Auth + Neon Postgres)
- Better-Auth for user authentication and session management
- Neon Serverless Postgres for storing user profiles and background information
- Background collection during signup process

### 3. Personalization Service (FastAPI + Neon Postgres)
- FastAPI service for adapting content based on user background
- Database for storing user preferences and personalization settings
- Integration with content delivery system

### 4. Translation Service (FastAPI + Translation APIs)
- Service for translating content to Urdu
- Integration with translation APIs (Google Cloud Translation, etc.)
- Caching for improved performance

## Troubleshooting Common Issues

### 1. GitHub Actions Build Failure

**Symptoms**: The build step in GitHub Actions fails
**Solutions**:
- Check the logs in the Actions tab for specific error messages
- Verify that all dependencies are properly specified in package.json
- Ensure that the build command works locally with `npm run build`

### 2. Features Not Working After Deployment

**Symptoms**: RAG chatbot, authentication, or other features don't work
**Solutions**:
- Verify that your backend service URLs are correctly configured
- Check browser console for CORS or network errors
- Ensure your backend services are properly deployed and accessible

### 3. Slow Performance

**Symptoms**: Pages load slowly or features respond slowly
**Solutions**:
- Optimize images and assets for web delivery
- Check that your backend services are deployed in locations with low latency
- Verify that your vector database is properly configured for fast retrieval

## Success Criteria

After deployment, verify that:

- [ ] Complete textbook is accessible at the GitHub Pages URL
- [ ] All modules (1-4) and capstone project are accessible
- [ ] RAG chatbot answers questions based only on textbook content
- [ ] User authentication system collects background information during signup
- [ ] Personalization features adapt content per chapter based on user background
- [ ] Urdu translation functionality available per chapter
- [ ] All content renders correctly with proper formatting
- [ ] Site meets accessibility standards (WCAG 2.1 AA)
- [ ] Performance metrics meet requirements (page load times, etc.)

## Post-Deployment Steps

1. **User Acceptance Testing**: Have beta users test the deployed textbook
2. **Performance Monitoring**: Monitor site performance and user engagement
3. **Content Updates**: Plan for regular content updates based on user feedback
4. **Feature Enhancement**: Plan for additional features based on user needs

## Next Steps

With the textbook successfully deployed to GitHub Pages:

1. Promote the textbook to students and educators
2. Gather user feedback for improvements
3. Add new content modules based on user needs
4. Enhance existing features based on usage data
5. Plan for real hardware integration and testing