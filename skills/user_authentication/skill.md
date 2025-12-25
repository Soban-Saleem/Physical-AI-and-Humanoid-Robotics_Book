# User Authentication Skill

**Skill Name**: User Authentication
**Version**: 1.0.0
**Author**: Panaversity Hackathon Team
**Created**: 2025-01-08
**Status**: Active

## Description

The User Authentication Skill handles user authentication, registration, and background information collection for the Physical AI & Humanoid Robotics textbook platform. This skill implements secure authentication using Better-Auth and collects user background information during signup.

## Capabilities

- Implement user signup and sign-in functionality
- Collect user background information (software and hardware experience)
- Securely store and manage user credentials
- Provide session management for authenticated users
- Protect platform features requiring authentication
- Support password reset functionality
- Handle authentication errors gracefully

## Inputs

- User credentials (email, password)
- Background information (software/hardware experience)
- Authentication requirements (protected routes, permissions)

## Outputs

- Authenticated user sessions
- User profile with background information
- Protected route access decisions
- Password reset tokens

## Dependencies

- Better-Auth for authentication system
- Neon Serverless Postgres for user data storage
- Security libraries for password hashing
- Validation libraries for input validation

## Usage

This skill is typically invoked by the Authentication Agent when implementing user authentication for the textbook platform.

## Configuration

- `collect_background`: Whether to collect background information during signup (default: true)
- `require_verification`: Whether to require email verification (default: false)
- `session_timeout`: Session timeout in minutes (default: 60)

## Performance Metrics

- Authentication operation time: < 500ms
- Login success rate: > 99%
- Security compliance: 100%