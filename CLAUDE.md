# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Common Development Commands

```bash
# Create virtual environment and install dependencies
invoke create-venv

# Install package in development mode (in existing venv)
invoke install

# Run tests with coverage
invoke test

# Lint and format code (uses ruff and mypy)
invoke lint

# Run full CI pipeline locally in Docker
invoke ci

# Build package
invoke build_package
```

Note: After modifying Python files, always run `ruff check --fix` and `ruff format` (covered by `invoke lint`).

## Project Architecture

**rox-control** is a Python library providing production-ready path tracking controllers for robotics applications.

### Key Components

- **Main Package**: `src/rox_control/` - Core control algorithms and interfaces
- **Tools**: `src/tools/` - Supporting simulation and visualization code (not packaged)
- **External References**: `temp/external/PythonRobotics/` - Educational reference implementations from PythonRobotics project

### Package Structure

- `src/rox_control/__init__.py` - Main package entry point
- `src/rox_control/version.py` - Version management via setuptools_scm
- `src/tools/bycicle_sim.py` - Bicycle kinematics simulator for testing controllers

### Dependencies

- **rox-vectors**: Core dependency for vector operations
- **uv**: Package manager for dependency resolution
- **invoke**: Task automation framework
- **ruff**: Linting and formatting
- **mypy**: Static type checking
- **pytest**: Testing framework

## Development Workflow

1. This project uses **uv** for package management instead of pip
2. All automation tasks are managed through **invoke** (see `tasks.py`)
3. Code must pass ruff linting, mypy type checking, and pytest tests
4. Versioning is handled automatically via git tags with setuptools_scm
5. CI/CD runs in Docker containers for consistency

## Testing

- Tests are in `tests/` directory
- Use `pytest` with coverage reporting
- Current test coverage includes smoke tests for imports and version checking
- Test configuration is in `pyproject.toml` under `[tool.pytest.ini_options]`

## Code Quality Standards

- Python 3.11+ required
- All functions must have type hints (enforced by mypy)
- Code formatting handled by ruff
- Pre-commit hooks available for automated checks
- Target line length and style defined in `pyproject.toml` ruff configuration

## References

The project builds upon concepts from PythonRobotics but focuses on production-ready implementations rather than educational examples. External reference code is available in `temp/external/` for comparison and learning.