# Physical AI & Humanoid Robotics Textbook Constitution

**Project**: Physical AI & Humanoid Robotics Textbook with Integrated Features

## Core Principles

### I. Educational Excellence
- All content must maintain the highest educational standards appropriate for the Physical AI & Humanoid Robotics curriculum
- Technical accuracy is paramount - all robotics, AI, and engineering concepts must be verified by authoritative sources
- Content must be accessible to students with varying backgrounds in software and hardware

### II. Spec-Driven Development
- All development follows the Spec-Kit Plus methodology with clear specs, plans, and tasks
- Every feature starts with a well-defined specification before implementation
- Changes to architecture require formal Architectural Decision Records (ADRs)

### III. Integrated Functionality
- All components (textbook, RAG chatbot, authentication, personalization, translation) must work seamlessly together
- Each feature must enhance the learning experience without compromising others
- Integration points between components must be well-defined and documented

### IV. Accessibility and Inclusion
- All content and interfaces must meet WCAG 2.1 AA standards
- Urdu translation capability must preserve technical accuracy
- Personalization features must adapt content appropriately based on user background

## Key Standards

### Content Quality Standards
- Writing Clarity: Flesch-Kincaid grade level 12-14 for undergraduate content, grade 16-18 for graduate content
- Technical Accuracy: All technical claims must be verifiable against at least one authoritative source (research paper, official documentation, or textbook)
- Citation Requirements: All non-common knowledge technical concepts must include citations to authoritative sources
- Originality: AI-generated content must be validated for originality with <5% similarity to existing sources (allowing for common technical terminology)
- Review Process: All content must undergo technical review by subject matter expert before publication

### Technical Standards
- Docusaurus framework for textbook publication
- FastAPI for backend services
- Neon Serverless Postgres for database storage
- Qdrant Cloud for vector storage in RAG system
- Better-Auth for user authentication and background collection

### Performance and Accessibility Standards
- Page Load Time: All textbook pages must load in <3 seconds on 3G connection (per Lighthouse standards)
- Accessibility: Achieve minimum 95% score on automated accessibility testing tools (axe-core)
- Chatbot Response Time: 95th percentile response time <5 seconds
- Mobile Responsiveness: Pass Google's Mobile-Friendly Test with 100% score
- Backend API Performance: 95th percentile response time <500ms for simple requests

### Quality Standards
- Code quality: All implementations must follow best practices and include comprehensive tests (minimum 90% code coverage)
- Security: All user data and authentication must follow security best practices (OWASP Top 10 compliance)
- Performance: All features must meet specified performance benchmarks above

### Source Verification and Review Process
- Technical Verification: All code examples must be tested and verified to work with specified hardware/software requirements
- Expert Review: Each module must undergo review by a domain expert with >5 years experience in the relevant field
- Peer Review: Content must be reviewed by at least one other team member before publication
- Continuous Validation: Implement automated checks for broken links, outdated information, and deprecated APIs
- Feedback Integration: Establish process to incorporate user feedback into content updates within 30 days

## Constraints

### Technical Constraints
- Deployment to GitHub Pages for frontend textbook
- Backend services must work with cloud infrastructure (Neon, Qdrant, etc.)
- All components must be compatible with Claude Code Subagents and Skills

### Content Constraints
- Must cover all specified curriculum modules: The Robotic Nervous System (ROS 2), The Digital Twin (Gazebo & Unity), The AI-Robot Brain (NVIDIA Isaac™), Vision-Language-Action (VLA)
- Content must be suitable for the specified hardware requirements (RTX workstations, Jetson kits, etc.)
- All technical explanations must be accurate and up-to-date with current practices

### Timeline and Submission Constraints
- Must be deployable to GitHub Pages with all features functional
- Must include RAG chatbot that answers questions based only on textbook content
- Must include user authentication with background collection
- Must include personalization features per chapter
- Must include Urdu translation capability per chapter

## Success Criteria

### Functional Success
- Complete textbook deployed to GitHub Pages with all curriculum content
- RAG chatbot successfully answers questions based on textbook content
- User authentication system collects background information during signup
- Personalization features adapt content based on user background
- Urdu translation functionality available per chapter
- All backend services (API, database, vector storage) properly deployed and connected

### Quality Success
- All content maintains technical accuracy and educational value
- All features meet accessibility standards (WCAG 2.1 AA)
- All components pass security review (OWASP Top 10 compliance)
- All components meet performance benchmarks
- Content originality score: <5% similarity to existing sources

### Integration Success
- All components work seamlessly together
- User experience is smooth across all features
- Backend services properly support all frontend functionality
- Personalization and translation features integrate smoothly with content

### Documentation Success
- All Architectural Decision Records (ADRs) properly documented
- All components have appropriate specifications, plans, and tasks
- Implementation follows Spec-Kit Plus methodology
- All Claude Code Subagents and Skills properly defined and documented

### Review and Validation Success
- All content passes technical review by domain experts
- All content passes peer review
- Automated validation checks pass (links, code examples, etc.)
- User acceptance testing shows >85% satisfaction rate