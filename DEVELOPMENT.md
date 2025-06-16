# Development

## Design philosophy

* Simulation and visualisation are decoupled.
* The system is synchronous, `.step(dt)` is used to calculate next step in time.
* Simulation runs as quickly as possible (no `sleep` delays) and produces a list of `RobotState`s.
* Visualisation uses simulation data to visualize it with plots or replay with liveplots, which use `sleep`.

## Feature roadmap

### Planned
**Static Visualization Tools**

* [feat_001] ✅ **COMPLETED** - Static visualization functions in [`src/tools/plot.py`](src/tools/plot.py)

* [feat_002] ✅ **COMPLETED** - Updated example to use plotting in [`examples/01_basic_simulation.py`](examples/01_basic_simulation.py)

**Basic waypoint follower**

NOTE: "track" term is used for a series of waypoints that a robot must follow. This was previusly
often named "path", which has a conflicting meaning with filepath. Therefore useage of "track" is preferred.

This functionality should go into `src/rox_control/tracks.py`.
For reference, see `temp/external/python-robotics/examples/pure_pursuit/pure_pursuit_2.py`

* [feat_003] ✅ **COMPLETED** - Track class for waypoint management in [`src/rox_control/tracks.py`](src/rox_control/tracks.py) with tests in [`tests/test_tracks.py`](tests/test_tracks.py)

* [feat_004] ✅ **COMPLETED** - Track generator functions in [`src/tools/tracks.py`](src/tools/tracks.py) with tests in [`tests/test_track_generators.py`](tests/test_track_generators.py)

* [feat_005] ✅ **COMPLETED** - Pure pursuit A controller in [`src/rox_control/controllers/pure_pursuit_a.py`](src/rox_control/controllers/pure_pursuit_a.py) with tests in [`tests/test_controllers.py`](tests/test_controllers.py)

* [feat_006] 🔄 **WIP** - Pure Pursuit A controller example in `examples/02_pure_pursuit_a.py`:
  - ✅ Basic implementation with timeout safety and track visualization
  - ✅ Clean control interface using `set_control_command(curvature, velocity)`
  - ✅ 20m x 20m square track with scaled parameters
  - ❌ **ISSUE**: Controller behavior shows poor path following in plots - needs debugging
  - **Status**: Implementation complete but controller tuning/debugging required

* [feat_007] - Animation debugging tool in `src/tools/animation.py`:
  - `animate_simulation(states, controller_outputs, track)` - step-by-step replay
  - Show robot position, heading, and front wheel position at each timestep
  - Visualize track waypoints and current target point from controller
  - Display controller debug info: lookahead point, projected position, angle error
  - Configurable playback speed for detailed analysis
  - Enable/disable different visualization layers (track, target, lookahead, etc.)
  - **Purpose**: Debug controller behavior and tune parameters visually


**Architectural Considerations:**
- **Module Organization:** ✅ Controllers organized in `src/rox_control/controllers/` directory structure
  - Each controller in separate file with descriptive name (e.g., `pure_pursuit_a.py`)
  - Exported with clear names via `__init__.py` (e.g., `PurePursuitA`)
  - Shared `ControlOutput` dataclass for consistent interface across controllers
- **Testing Strategy:** ✅ Comprehensive unit tests with edge cases implemented
  - Core library features have full test coverage (empty tracks, single waypoint, malformed data)
  - Code in `tools` may have less strict testing requirements
- **Type Safety:** ✅ Full mypy compliance achieved with proper type annotations
- **Documentation:** ✅ One-line docstrings added for all classes and non-trivial functions



### Not planned yet

**Animation Tools**




* Create `examples/02_animate.py`
    - Use `examples/01_basic_simulation.py` as a base (import functions).
    - use `RobotState` objects to create an animation.



*  Extend `src/tools/plot.py` with additional plot types
    - Trajectory analysis plots (curvature, acceleration profiles)
    - Multi-simulation comparison capabilities

* Extend `src/tools/animation.py` with interactive features
    - Play/pause controls, speed adjustment
    - Real-time parameter display overlay

**Integration & Production Features**

* [feat_008] Add visualization examples to package documentation

* [feat_009] Create configuration-driven simulation scenarios

* [feat_010] Add export capabilities (PNG/PDF for plots, GIF/MP4 for animations)


## Tooling

### Local Development
```bash
# Create virtual environment
invoke create-venv

# Run tests
invoke test

# Lint and format code
invoke lint
```

### VS Code DevContainer
Open in VS Code and select "Reopen in Container" for a pre-configured development environment with all tools and extensions.

### CI/CD

* **Local CI**: Run `invoke ci` to test in Docker container


* **Dependencies**: `uv` for fast package management
* **Automation**: `invoke` - run `invoke -l` to list available commands
* **Versioning**: `setuptools_scm` with git tags
* **Linting**: `ruff` for fast linting and formatting
* **Type checking**: `mypy` for static analysis

## Project Structure

* `src/rox_control/` - Application code
* `src/tools/` - Supporting simulation and visualization code (not packaged)
* `tasks.py` - Automation tasks via invoke
* `pyproject.toml` - Modern Python packaging configuration
