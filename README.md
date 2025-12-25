# Physical AI & Humanoid Robotics Textbook

This repository contains the complete implementation of the Physical AI & Humanoid Robotics textbook with all integrated features including RAG chatbot, authentication, personalization, and Urdu translation.

## Project Status

✅ **Repository Setup**: Complete
✅ **Textbook Content**: Complete (4 modules + capstone)
✅ **GitHub Actions Workflow**: Configured and operational
✅ **GitHub Pages**: Deployed at https://soban-saleem.github.io/Physical-AI-and-Humanoid-Robotics_Book/
✅ **All Features Functional**: RAG chatbot, authentication, personalization, translation
✅ **Claude Code Subagents**: Complete (10 specialized agents)
✅ **Claude Code Skills**: Complete (11 reusable skills)
✅ **Spec-Kit Plus Methodology**: Fully implemented
✅ **Architecture Decision Records**: Complete documentation
✅ **Complete Integration**: All components work seamlessly together

## Deployment Status

The textbook has been successfully deployed to GitHub Pages at: https://soban-saleem.github.io/Physical-AI-and-Humanoid-Robotics_Book/

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
- **Accessible**: WCAG 2.1 AA compliant

## Implementation Architecture

- **Frontend**: Docusaurus-based textbook with interactive elements
- **Backend Services**: FastAPI for chatbot, authentication, and personalization
- **Database**: Neon Serverless Postgres for user data and content metadata
- **Vector Storage**: Qdrant Cloud for RAG system embeddings
- **AI Integration**: OpenAI APIs for vision-language-action models
- **Simulation**: Gazebo and NVIDIA Isaac Sim for digital twin

## Development Structure

- `textbook-site/` - Docusaurus-based textbook implementation
- `specs/` - Feature specifications following Spec-Kit Plus methodology
- `agents/` - Claude Code agents for various textbook components
- `skills/` - Reusable skills for textbook implementation
- `.github/workflows/` - GitHub Actions for automated deployment

## Deployment Process

The deployment is fully automated through GitHub Actions. When changes are pushed to the master branch, the following occurs:
1. GitHub Actions triggers the deployment workflow
2. Dependencies are installed and the Docusaurus site is built
3. The built site is deployed to the `gh-pages` branch
4. GitHub Pages serves the site at the configured URL

## Contributing

This project follows the Spec-Driven Development methodology. All changes should be made following the specification → plan → tasks workflow.

## License

This project is part of the Panaversity HACKATHON-Book initiative.