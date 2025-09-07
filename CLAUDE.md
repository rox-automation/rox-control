# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Installation Options

```bash
# Install core controllers only (minimal dependencies)
pip install rox-control

# Install with visualization and development tools
pip install rox-control[tools]
```

## Common Development Commands

```bash
# Create virtual environment and install dependencies
invoke create-venv

# Install package in development mode (in existing venv)
invoke install

# Run tests with coverage
invoke test

# Run a single test file
pytest tests/test_specific_file.py

# Lint and format code (uses ruff and mypy)
invoke lint

# Run full CI pipeline locally in Docker
invoke ci

# Build package
invoke build_package

# Generate UML diagrams from source code
invoke uml

# Clean untracked files (interactive)
invoke clean

# Release to PyPI (requires proper git tag)
invoke release
```

Note: After modifying Python files, always run `ruff check --fix` and `ruff format` (covered by `invoke lint`).

## Project Architecture

**rox-control** is a Python library providing production-ready path tracking controllers for robotics applications.

### Key Components

- **Main Package**: `src/rox_control/` - Core control algorithms and interfaces
- **Tools**: `src/rox_control/tools/` - Simulation and visualization tools (optional dependency)
- **External References**: `temp/external/PythonRobotics/` - Educational reference implementations from PythonRobotics project

### Package Structure

- `src/rox_control/__init__.py` - Main package entry point exposing `Track` and `__version__`
- `src/rox_control/track.py` - Track/waypoint management functionality  
- `src/rox_control/controllers/` - Path tracking controllers (exports `PurePursuitA`, `ControlOutput`)
- `src/rox_control/tools/` - Optional simulation, visualization, and bicycle kinematics tools
- `src/rox_control/version.py` - Version management via setuptools_scm
- `examples/` - Usage demonstrations including basic simulation and pure pursuit examples

### Dependencies

#### Core Dependencies (minimal installation)
- **rox-vectors**: Core dependency for vector operations
- **numpy**: Numerical computing foundation

#### Optional Dependencies (`[tools]` extra)
- **matplotlib**: Required for visualization and plotting tools

#### Development Dependencies  
- **uv**: Package manager for dependency resolution (not pip)
- **invoke**: Task automation framework
- **ruff**: Linting and formatting
- **mypy**: Static type checking with strict untyped definitions disallowed
- **pytest**: Testing framework with coverage reporting

## Development Workflow

1. This project uses **uv** for package management instead of pip
2. All automation tasks are managed through **invoke** (see `tasks.py`)
3. Code must pass ruff linting, mypy type checking, and pytest tests
4. Versioning is handled automatically via git tags with setuptools_scm
5. CI/CD runs in Docker containers for consistency

## Package Usage

### Core Controllers (Always Available)
```python
from rox_control import Track
from rox_control.controllers import PurePursuitA, ControlOutput
```

### Development Tools (Requires `[tools]` extra)
```python
from rox_control.tools import BicycleModel, RobotState
from rox_control.tools import plot_simulation_data, present_results
from rox_control.tools import generate_track, rectangular_track
```

### Graceful Degradation
Tools provide helpful error messages when matplotlib is not installed:
```python
from rox_control.tools import plot_simulation_data
# Raises ImportError with installation instructions if matplotlib missing
```

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

### Docstring Style Guidelines

- **Classes**: Single descriptive line explaining the class purpose
- **Methods without return values**: Single descriptive line explaining what the method does
- **Methods with return values**: Two lines:
  - First line: Descriptive explanation of what the method does
  - Second line: `Returns: <description of return value>`
- **Type hints should be descriptive enough** - avoid verbose argument descriptions
- **Use modern Python 3.10+ union syntax** (`Type | None` instead of `Optional[Type]`)

Example:
```python
def calculate_distance(self, point_a: Vector, point_b: Vector) -> float:
    """Calculate Euclidean distance between two points.

    Returns: Distance in meters.
    """
```

## References

The project builds upon concepts from PythonRobotics but focuses on production-ready implementations rather than educational examples. External reference code is available in `temp/external/` for comparison and learning.
