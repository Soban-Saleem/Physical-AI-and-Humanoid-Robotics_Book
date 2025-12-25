# Skills Directory for Physical AI & Humanoid Robotics Textbook

This directory contains reusable skills that can be leveraged by Claude Code Subagents to build the Physical AI & Humanoid Robotics textbook platform.

## Directory Structure

Each skill follows this structure:
```
skills/
├── skill_name/
│   ├── skill.md          # Skill description and capabilities
│   ├── license.txt       # License information
│   └── scripts/          # Implementation scripts
│       └── *.sh          # Shell scripts for skill execution
```

## Available Skills

### Content Generation
- **Purpose**: Generate educational content for the textbook
- **Capabilities**: Curriculum-based content creation, difficulty adaptation, technical accuracy validation

### Textbook Formatting
- **Purpose**: Format content for Docusaurus compatibility
- **Capabilities**: Markdown formatting, heading hierarchy, accessibility compliance

### API Development
- **Purpose**: Create FastAPI backend services
- **Capabilities**: Endpoint generation, model creation, documentation, testing

### Database Operations
- **Purpose**: Manage database schemas and operations
- **Capabilities**: Schema design, ORM models, migrations, indexing

### User Authentication
- **Purpose**: Handle user authentication and background collection
- **Capabilities**: Signup, signin, background collection, session management

### Content Personalization
- **Purpose**: Adapt content based on user background
- **Capabilities**: Profile analysis, content adaptation, personalization engine

### Language Translation
- **Purpose**: Translate content to Urdu
- **Capabilities**: Content translation, caching, UI integration

### Vector Storage
- **Purpose**: Manage vector storage for RAG system
- **Capabilities**: Content ingestion, similarity search, embedding management

### Chat Interface
- **Purpose**: Create integrated chatbot UI
- **Capabilities**: Chat interface, message handling, context management

## Usage

These skills are designed to be used by Claude Code Subagents to perform complex tasks in the textbook creation process. Each skill can be invoked independently or as part of a larger agent workflow.