# ADR-002: Educational Content Structure and Progression

**Status**: Accepted  
**Date**: 2025-01-08

## Context

The Physical AI & Humanoid Robotics textbook must balance theoretical knowledge with hands-on practical experience. The content structure needs to accommodate different learning styles while ensuring students can progress from basic concepts to complex implementations. The course has four distinct modules that build toward a capstone autonomous humanoid project, requiring a pedagogically sound approach to content organization.

## Decision

We will implement a modular content structure with progressive complexity using the following approach:

- **Modular Design**: Content organized into distinct modules (The Robotic Nervous System, The Digital Twin, The AI-Robot Brain, Vision-Language-Action)
- **Hands-on Projects**: Each module includes hands-on exercises that build toward the capstone project
- **Progressive Disclosure**: Complex topics introduced gradually with foundational concepts first
- **Hardware Validation**: Code examples designed to work across simulation and real hardware
- **Assessment Integration**: Built-in validation points to measure learning outcomes

## Alternatives Considered

1. **Linear Structure**: Sequential approach where each topic builds directly on the previous one
   - Pros: Simple to follow, clear learning path
   - Cons: Less flexible, difficult to accommodate different learning paces, limited modularity

2. **Topic-based Structure**: Organized by technical topics rather than learning progression
   - Pros: Comprehensive coverage of each topic, good for reference
   - Cons: May not align with skill-building progression, potentially overwhelming for beginners

3. **Project-only Approach**: Focus primarily on projects with minimal theoretical content
   - Pros: Highly practical, engaging for hands-on learners
   - Cons: May lack foundational understanding, difficult for complex concepts

## Consequences

**Positive:**
- Students can start with basic concepts and advance to complex implementations
- Measurable progress indicators through hands-on projects
- Accommodates different learning styles and paces
- Clear pathway to capstone autonomous humanoid project
- Validation of learning through practical application

**Negative:**
- More complex content organization and maintenance
- Requires careful coordination between modules
- Potential for students to get lost in complexity
- Higher development effort for integrated content

## References

- plan.md: Architecture Overview and Implementation Order sections
- research.md: Educational Content Structure and Hands-on Projects decisions
- spec.md: Core learning outcomes and hands-on projects requirements