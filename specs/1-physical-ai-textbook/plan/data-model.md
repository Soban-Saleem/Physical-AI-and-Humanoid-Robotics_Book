# Data Model for Physical AI & Humanoid Robotics Textbook

**Created**: 2025-01-08
**Project**: Physical AI & Humanoid Robotics Textbook

## Core Entities

### Textbook Module
- **id**: string (UUID) - Unique identifier for the module
- **title**: string - Descriptive title of the module
- **description**: string - Brief overview of the module content
- **content**: string (Markdown) - Main content with embedded code examples
- **prerequisites**: array of strings - List of required knowledge areas
- **learning_objectives**: array of strings - Specific learning objectives
- **duration_hours**: number - Estimated time to complete the module
- **difficulty_level**: enum (beginner, intermediate, advanced) - Complexity level
- **technology_stack**: array of strings - Technologies covered in the module
- **exercises**: array of Exercise objects - Hands-on exercises
- **project_component**: ProjectComponent object - Capstone project component
- **validation_steps**: array of strings - Hardware validation procedures
- **created_at**: datetime - Timestamp of creation
- **updated_at**: datetime - Timestamp of last update
- **author**: string - Author of the module

### Code Example
- **id**: string (UUID) - Unique identifier for the example
- **module_id**: string (UUID) - Reference to parent module
- **title**: string - Descriptive title of the example
- **description**: string - Explanation of the code purpose
- **language**: string (default: "python") - Programming language
- **code**: string - Source code content
- **requirements**: string - Dependencies needed
- **expected_output**: string - Expected results when executed
- **hardware_requirements**: array of strings - Specific hardware needed for validation
- **validation_status**: enum (pending, validated, failed) - Status of hardware validation
- **validation_notes**: string - Notes from validation process
- **created_at**: datetime - Timestamp of creation

### Exercise
- **id**: string (UUID) - Unique identifier for the exercise
- **module_id**: string (UUID) - Reference to parent module
- **title**: string - Descriptive title of the exercise
- **description**: string - Detailed exercise description
- **difficulty_level**: enum (beginner, intermediate, advanced) - Complexity level
- **instructions**: string (Markdown) - Step-by-step instructions
- **expected_outcome**: string - What the student should achieve
- **validation_criteria**: array of strings - How to verify completion
- **hints**: array of strings - Optional guidance for students
- **resources**: array of strings - Additional resources or references
- **created_at**: datetime - Timestamp of creation

### Project Component
- **id**: string (UUID) - Unique identifier for the project component
- **module_id**: string (UUID) - Reference to parent module
- **title**: string - Project component title
- **description**: string - Detailed project description
- **requirements**: array of strings - Technical requirements
- **implementation_steps**: array of strings - Step-by-step implementation guide
- **validation_criteria**: array of strings - How to verify successful completion
- **integration_points**: array of strings - How project connects to other modules
- **expected_completion_time**: number - Estimated time in hours
- **dependencies**: array of strings - Prerequisites from other modules
- **deliverables**: array of strings - What students must submit
- **created_at**: datetime - Timestamp of creation

### Hardware Configuration
- **id**: string (UUID) - Unique identifier for the configuration
- **name**: string - Descriptive name (e.g., "RTX 4080+ Workstation")
- **type**: enum (workstation, edge_device, robot) - Category of hardware
- **specifications**: object - Technical specifications
- **supported_modules**: array of strings - Modules that can be validated on this hardware
- **setup_instructions**: string (Markdown) - How to configure the hardware
- **validation_status**: enum (verified, unverified, partial) - Validation status
- **last_verified**: datetime - Last validation timestamp
- **notes**: string - Additional notes about the configuration
- **created_at**: datetime - Timestamp of creation

## Relationships

### Textbook Module Relationships
- Contains many Code Examples
- Contains many Exercises
- Has one Project Component
- May be part of a Module Sequence (prerequisites/dependencies)

### Code Example Relationships
- Belongs to one Textbook Module
- May be referenced by Validation Tests

### Exercise Relationships
- Belongs to one Textbook Module
- May have multiple Submissions (from students)

### Project Component Relationships
- Belongs to one Textbook Module
- May integrate with other Project Components (from other modules)

## Validation Rules

### Textbook Module
- Title must be 5-100 characters
- Content must be valid Markdown
- At least one learning objective required
- Duration must be a positive number
- Difficulty level must be specified

### Code Example
- Module ID must reference an existing module
- Code must be valid for the specified language
- Hardware requirements must be specified if validation is expected
- Validation status must be updated after testing

### Exercise
- Module ID must reference an existing module
- Instructions must be provided
- Validation criteria must be specific and measurable

### Project Component
- Module ID must reference an existing module
- At least one implementation step required
- Validation criteria must be specific and measurable

### Hardware Configuration
- Name must be unique
- Type must be specified
- Specifications must be provided