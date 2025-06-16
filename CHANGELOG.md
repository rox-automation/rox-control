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
- **Track generator functions** (feat_004)
  - Main `generate_track(track_type: str, **kwargs) -> Track` function
  - Square track generation with 5 waypoints (4 corners + closing point)
  - Circular track generation with configurable radius, center, and resolution
  - Figure-8 track generation with configurable size and resolution
  - Parameter validation with clear error messages for invalid inputs
  - Full integration with existing Track class from feat_003
  - Comprehensive test suite with 18 test cases covering all track types and edge cases
  - 100% test coverage (44/44 statements)
- **Pure Pursuit A Controller** (feat_005)
  - Velocity-projected pure pursuit algorithm with proportional control
  - `PurePursuitA` class with configurable parameters (lookahead, gain, speed)
  - `ControlOutput` dataclass with structured control commands and debug info
  - `set_track()` and `control()` methods for clean stateful interface
  - Projects robot future position using velocity vector before path projection
  - Uses `point_on_line` for accurate segment projection and lookahead targeting
  - Handles track completion and edge cases (empty tracks, beyond end)
  - Full integration with existing `Track` and `RobotState` types
  - Comprehensive test suite with 16 test methods covering unit and integration scenarios
  - 96% test coverage (47/49 statements)

### Changed
- **Controller organization**: Moved from single file to modular directory structure
  - Controllers now organized in `src/rox_control/controllers/` directory
  - Each controller implemented in separate file (e.g., `pure_pursuit_a.py`)
  - Exported with descriptive names via `__init__.py` (e.g., `PurePursuitA`)
  - Shared `ControlOutput` dataclass for consistent interface across controllers

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
