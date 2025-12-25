# Database Operations Skill

**Skill Name**: Database Operations
**Version**: 1.0.0
**Author**: Panaversity Hackathon Team
**Created**: 2025-01-08
**Status**: Active

## Description

The Database Operations Skill is responsible for managing database schemas, operations, and connections for the Physical AI & Humanoid Robotics textbook platform. This skill handles schema design, ORM model generation, migrations, and performance optimization for Neon Serverless Postgres.

## Capabilities

- Design comprehensive database schemas for all platform entities
- Generate SQLAlchemy/SQLModel models for ORM usage
- Create database migration scripts for schema changes
- Implement proper relationships between entities
- Optimize database performance with appropriate indexing
- Ensure data integrity through proper constraints

## Inputs

- Entity definitions (JSON format)
- Schema requirements (constraints, relationships, indexes)
- Migration specifications (old schema, new schema)

## Outputs

- Database schema definitions
- SQLAlchemy/SQLModel models
- Migration scripts
- Performance optimization recommendations

## Dependencies

- SQLAlchemy or SQLModel
- Alembic for migrations
- Neon Serverless Postgres
- Database validation tools

## Usage

This skill is typically invoked by the Database Schema Agent when designing and managing the platform's database structure.

## Configuration

- `enable_migrations`: Whether to generate migration scripts (default: true)
- `validate_relationships`: Whether to validate entity relationships (default: true)
- `optimize_indexes`: Whether to suggest performance optimizations (default: true)

## Performance Metrics

- Schema generation time: < 10 seconds
- Migration success rate: > 99%
- Index optimization effectiveness: > 90%