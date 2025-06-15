# Changelog
All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/), and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## Unreleased

### Added
- **Track class** for waypoint management (feat_003)
  - Inherits from `UserList` for list-like behavior with full indexing support
  - Uses `rox_vectors.Vector` for waypoints with complete type annotation coverage
  - Core tracking logic with `find_next_idx(xy: Vector) -> int` method
  - `target_reached` property for track completion detection
  - Automatic Vector conversion from tuples/lists in constructor
  - Validation ensures minimum 2 waypoints requirement
  - Private `_next_idx` attribute for internal state management
  - Comprehensive test suite with 20 test cases covering edge cases
  - 100% test coverage (26/26 statements)

## [0.1.0] - 2025-06-15

### Added
- **BicycleModel** class with realistic bicycle kinematics simulation
  - Rate-limited steering and acceleration 
  - Configurable wheelbase, max steering angle, and acceleration limits
  - State tracking with time progression
- **Geometric methods** for bicycle model
  - `get_front_wheel_pos()` - calculates front wheel position using wheelbase
  - `get_projected_path()` - predicts future path based on current steering angle
- **Static visualization system** (feat_001, feat_002)
  - `plot_simulation_results(states, model)` function with 2-column layout
  - Left panel: XY trajectory with rear/front wheel traces and start/end markers
  - Right panel: Steering angle and velocity vs time (stacked subplots)
  - Uses model geometric methods for accurate wheel positioning
- **Example simulation** (`examples/01_basic_simulation.py`)
  - Demonstrates bicycle model usage with realistic maneuvers
  - Integrated plotting functionality with graceful matplotlib handling
- **Comprehensive test suite**
  - 20 test cases with 100% coverage for bicycle model
  - Smoke tests for imports and basic functionality
  - All geometric methods thoroughly tested

### Changed
- **Simplified API**: Removed `BicycleModel.reset()` method in favor of direct state assignment
- **Dependencies**: Added matplotlib as dev dependency for visualization
- **Performance**: Optimized smoke tests for CI/CD speed (0.39s total test time)

### Infrastructure
- Python 3.11+ with modern type hints
- UV package manager for fast dependency resolution
- Ruff linting and formatting with mypy type checking
- Invoke task automation with pytest testing
- Setuptools-scm for automatic versioning from git tags
- CI-ready test suite with coverage reporting
