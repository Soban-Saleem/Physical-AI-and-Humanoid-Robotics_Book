# Quickstart Guide: Deploy Book to GitHub Pages

**Created**: 2025-01-08
**Project**: Physical AI & Humanoid Robotics Textbook Deployment

## Prerequisites

Before deploying the Physical AI & Humanoid Robotics textbook to GitHub Pages, ensure you have:

### System Requirements
- Ubuntu 22.04 LTS or similar Linux environment
- Node.js 18+ and npm
- Git installed and configured
- GitHub account with repository access
- Access to backend services (Neon, Qdrant, etc.)

### Project Requirements
- Complete textbook content in Docusaurus format
- All integrated features (RAG chatbot, authentication, personalization, translation) tested
- Valid configuration files for backend service connections
- Properly formatted images and assets

## Step-by-Step Deployment Process

### Step 1: Repository Setup
1. Fork or clone the textbook repository to your GitHub account
2. Ensure you have write permissions to the repository
3. Verify that GitHub Pages is enabled in repository settings:
   - Go to repository Settings → Pages
   - Set Source to "Deploy from a branch"
   - Select branch "gh-pages" and folder "/(root)"

### Step 2: Environment Configuration
1. Set up your local development environment:
   ```bash
   # Clone the repository
   git clone https://github.com/[your-username]/[repository-name].git
   cd [repository-name]
   
   # Install dependencies
   npm install
   ```

2. Configure environment variables for backend services:
   ```bash
   # Create .env file with backend service endpoints
   echo "REACT_APP_RAG_API_URL=https://your-rag-service.com/api" >> .env
   echo "REACT_APP_AUTH_API_URL=https://your-auth-service.com/api" >> .env
   echo "REACT_APP_PERSONALIZATION_API_URL=https://your-personalization-service.com/api" >> .env
   echo "REACT_APP_TRANSLATION_API_URL=https://your-translation-service.com/api" >> .env
   ```

### Step 3: Docusaurus Configuration
1. Update the `docusaurus.config.js` file with your deployment settings:
   ```javascript
   // docusaurus.config.js
   const config = {
     // Site metadata
     title: 'Physical AI & Humanoid Robotics Textbook',
     tagline: 'Full-System Integration Skills for Embodied AI Systems',
     favicon: 'img/favicon.ico',
     
     // GitHub Pages deployment configuration
     url: 'https://[your-username].github.io',
     baseUrl: '/[repository-name]/',
     
     // GitHub repository information
     organizationName: '[your-username]',
     projectName: '[repository-name]',
     
     // ... rest of configuration
   };
   ```

2. Verify that all content paths are correct and all modules are properly linked in `sidebars.js`

### Step 4: Build the Site
1. Test the build process locally:
   ```bash
   # Build the site
   npm run build
   
   # Serve locally to test
   npm run serve
   ```

2. Verify that all pages load correctly and all features function as expected

### Step 5: Set Up GitHub Actions
1. Create the GitHub Actions workflow file at `.github/workflows/deploy.yml`:

```yaml
name: Deploy to GitHub Pages

on:
  push:
    branches: [main]
  workflow_dispatch:

permissions:
  contents: read
  pages: write
  id-token: write

concurrency:
  group: "pages"
  cancel-in-progress: true

jobs:
  build:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v3
      - uses: actions/setup-node@v3
        with:
          node-version: 18
          cache: npm

      - name: Install dependencies
        run: npm ci

      - name: Build website
        run: npm run build

      - name: Setup Pages
        uses: actions/configure-pages@v3

      - name: Upload artifact
        uses: actions/upload-pages-artifact@v1
        with:
          path: build

  deploy:
    environment:
      name: github-pages
      url: ${{ steps.deployment.outputs.page_url }}
    runs-on: ubuntu-latest
    needs: build
    steps:
      - name: Deploy to GitHub Pages
        id: deployment
        uses: actions/deploy-pages@v1
```

2. Ensure the workflow has appropriate permissions in GitHub repository settings

### Step 6: Deployment
1. Commit your changes:
   ```bash
   git add .
   git commit -m "Configure GitHub Pages deployment"
   git push origin main
   ```

2. The GitHub Actions workflow will automatically build and deploy the site
3. Monitor the deployment progress in the "Actions" tab of your repository

### Step 7: Post-Deployment Verification
1. Access your deployed site at: `https://[your-username].github.io/[repository-name]/`
2. Verify that all textbook modules are accessible
3. Test all integrated features (chatbot, personalization, translation)
4. Validate that all links and navigation work correctly
5. Check that the site is responsive on different devices

## Troubleshooting Common Issues

### Build Failures
- **Issue**: Docusaurus build fails with dependency errors
- **Solution**: Ensure all dependencies are properly installed with `npm ci`

### Missing Content
- **Issue**: Some pages or content not showing in deployed version
- **Solution**: Verify that all content files are properly committed and paths are correct

### Feature Malfunctions
- **Issue**: RAG chatbot or other features not working in deployed version
- **Solution**: Check that backend service URLs are correctly configured and accessible from deployed site

### Performance Issues
- **Issue**: Site loading slowly
- **Solution**: Optimize images and assets, verify build process is minifying code properly

## Updating the Deployment

After the initial deployment, any changes pushed to the `main` branch will automatically trigger a new deployment. To update content:

1. Make changes to your local repository
2. Test changes locally with `npm run start`
3. Commit and push changes to main branch
4. Monitor GitHub Actions for successful deployment
5. Verify updates on the deployed site

## Custom Domain Setup (Optional)

If you want to use a custom domain:

1. In your GitHub repository Settings → Pages, add your custom domain
2. Add a CNAME file to your repository root with your domain name
3. Configure DNS settings with your domain registrar to point to GitHub Pages

## Backend Service Integration

Remember that GitHub Pages only hosts static content. Dynamic features like the RAG chatbot, authentication, and personalization require separate backend services. Ensure these services are deployed and accessible via the configured endpoints before deploying the static site.