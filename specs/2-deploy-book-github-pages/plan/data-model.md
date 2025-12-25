# Data Model: Deploy Book to GitHub Pages

**Created**: 2025-01-08
**Project**: Physical AI & Humanoid Robotics Textbook Deployment

## Core Entities

### Deployment Configuration
- **id**: string (UUID) - Unique identifier for the deployment configuration
- **site_name**: string - Name of the deployed site (e.g., "Physical AI & Humanoid Robotics Textbook")
- **repository**: string - GitHub repository for the site (e.g., "panaversity/physical-ai-textbook")
- **branch**: string - Branch to deploy from (default: "main")
- **build_command**: string - Command to build the site (default: "npm run build")
- **output_directory**: string - Directory containing built site (default: "./build")
- **backend_endpoints**: object - URLs for backend services
  - `rag_api_url`: string - URL for RAG chatbot API
  - `auth_api_url`: string - URL for authentication API
  - `personalization_api_url`: string - URL for personalization API
  - `translation_api_url`: string - URL for translation API
- **environment_variables**: object - Variables for build process
  - `GITHUB_TOKEN`: string - Token for GitHub Pages deployment
  - `BACKEND_API_KEY`: string - API key for backend services
- **created_at**: datetime - Timestamp of creation
- **updated_at**: datetime - Timestamp of last update
- **status**: enum (active, inactive, suspended) - Current status of deployment

### Deployment Status
- **id**: string (UUID) - Unique identifier for the deployment
- **deployment_id**: string - GitHub Pages deployment ID
- **status**: enum (queued, building, deployed, failed, cancelled) - Current status
- **commit_sha**: string - Commit hash for the deployment
- **trigger**: enum (manual, automatic) - What triggered the deployment
- **start_time**: datetime - Time deployment started
- **end_time**: datetime - Time deployment completed
- **logs**: array of strings - Deployment logs
- **error_message**: string - Error details if deployment failed
- **site_url**: string - URL of deployed site
- **user_id**: string (UUID) - ID of user who triggered deployment

### Build Artifact
- **id**: string (UUID) - Unique identifier for the build artifact
- **deployment_id**: string (UUID) - Reference to deployment
- **file_path**: string - Path to the built file
- **file_size**: integer - Size of file in bytes
- **checksum**: string - SHA-256 checksum of file
- **compression_type**: enum (none, gzip, brotli) - Compression applied
- **uploaded_at**: datetime - Time artifact was uploaded
- **valid**: boolean - Whether artifact passed validation

### Feature Configuration
- **id**: string (UUID) - Unique identifier for feature configuration
- **feature_name**: string - Name of the feature (e.g., "chatbot", "authentication", "personalization", "translation")
- **enabled**: boolean - Whether feature is enabled in deployment
- **configuration**: object - Feature-specific configuration parameters
- **deployment_id**: string (UUID) - Reference to deployment
- **created_at**: datetime - Timestamp of creation

## Relationships

### Deployment Configuration Relationships
- Has many Deployment Status records (one-to-many)
- Has many Build Artifacts (one-to-many)
- Has many Feature Configurations (one-to-many)

### Deployment Status Relationships
- Belongs to one Deployment Configuration
- Has many Build Artifacts (one-to-many)

## Validation Rules

### Deployment Configuration
- site_name must be 3-100 characters
- repository must follow format "owner/name"
- branch must be alphanumeric with allowed special characters (-, _, .)
- backend_endpoints must be valid URLs
- environment_variables must be properly formatted

### Deployment Status
- status must be one of the allowed values
- commit_sha must be a valid Git SHA
- start_time must be before end_time if deployment completed
- site_url must be a valid URL

### Build Artifact
- file_size must be greater than 0
- checksum must be a valid SHA-256 hash
- deployment_id must reference an existing deployment

### Feature Configuration
- feature_name must be one of the supported features
- configuration must match the expected schema for the feature
- deployment_id must reference an existing deployment

## API Endpoints

### Deployment Configuration Management
- GET /api/deployments/config - Get deployment configuration
- PUT /api/deployments/config - Update deployment configuration
- DELETE /api/deployments/config - Remove deployment configuration

### Deployment Status Management
- GET /api/deployments/status - Get deployment status
- POST /api/deployments/status - Create new deployment status record
- GET /api/deployments/status/{id} - Get specific deployment status

### Build Artifact Management
- GET /api/deployments/artifacts - Get list of build artifacts
- GET /api/deployments/artifacts/{id} - Get specific artifact
- POST /api/deployments/artifacts/validate - Validate build artifacts

### Feature Configuration Management
- GET /api/features/config - Get feature configurations
- PUT /api/features/config/{feature_name} - Update feature configuration
- GET /api/features/config/{feature_name} - Get specific feature configuration