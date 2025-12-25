# Physical AI & Humanoid Robotics Textbook

This repository contains the Physical AI & Humanoid Robotics textbook with integrated features including RAG chatbot, authentication, personalization, and Urdu translation.

## Project Status

✅ **Repository Setup**: Complete  
✅ **Textbook Content**: Complete (4 modules + capstone)  
✅ **GitHub Actions Workflow**: Configured  
✅ **GitHub Pages**: Enabled  

## Deployment Status

The textbook has been successfully pushed to this repository and the GitHub Pages deployment workflow is configured. To complete the deployment:

1. Verify that GitHub Actions is enabled for this repository
2. Check that the workflow in `.github/workflows/deploy.yml` has run successfully
3. The site should be available at: https://soban-saleem.github.io/Physical-AI-and-Humanoid-Robotics_Book/

## Textbook Structure

The textbook is organized into 4 core modules plus a capstone project:

1. **Module 1**: The Robotic Nervous System (ROS 2)
2. **Module 2**: The Digital Twin (Gazebo & Unity) 
3. **Module 3**: The AI-Robot Brain (NVIDIA Isaac™)
4. **Module 4**: Vision-Language-Action (VLA)
5. **Capstone**: Autonomous Humanoid Project

## Integrated Features

- **RAG Chatbot**: Answers questions based only on textbook content
- **Authentication**: User signup with background collection
- **Personalization**: Content adaptation per chapter based on user background
- **Urdu Translation**: Translation per chapter
- **Responsive Design**: Works on mobile, tablet, and desktop

## Development Structure

- `textbook-site/` - Docusaurus-based textbook implementation
- `specs/` - Feature specifications following Spec-Kit Plus methodology
- `agents/` - Claude Code agents for various textbook components
- `skills/` - Reusable skills for textbook implementation

## Next Steps

1. Go to the Actions tab in this repository to verify the deployment workflow completed successfully
2. Visit the deployed site at the URL above
3. Test all textbook features including navigation, chatbot, and interactive elements
4. Verify that all modules are accessible and properly formatted

## Contributing

This project follows the Spec-Driven Development methodology. All changes should be made following the specification → plan → tasks workflow.