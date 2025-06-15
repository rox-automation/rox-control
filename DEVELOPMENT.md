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

* [feat_006] - example of Pure Pursuit A controller in `examples/02_pure_pursuit_a.py`:
  - run simulation and display resulting plot
  - use square track (20x5 meters)
  - the code should function like `temp/external/python-robotics/examples/pure_pursuit/pure_pursuit_2.py`, but use code implementeed in features 3,4,5.
  - example should look like `examples/01_basic_simulation.py`. Use similar style. Avoid duplicating code, if required, move to separate module, in `tools`. For example `present_results` function is a good candidate.

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

* Create `src/tools/animation.py` with live animation functions
    - `animate_simulation(states: list[RobotState])` - replay with matplotlib animation
    - Progressive state replay with configurable playbook speed
    - Show projected path curve for front wheel during replay
    - Reference: `temp/external/python-robotics/temp/mpl_visualizer.py:369-443`


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

