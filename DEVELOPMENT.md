# Development

## Design philosophy

* Simulation and visualisation are decoupled.
* The system is synchronous, `.step(dt)` is used to calculate next step in time.
* Simulation runs as quickly as possible (no `sleep` delays) and produces a list of `RobotState`s.
* Visualisation uses simulation data to visualize it with plots or replay with liveplots, which use `sleep`.

## Feature roadmap

### Planned
**Static Visualization Tools**

* [feat_001] ✅ **COMPLETED** - Create `src/tools/plot.py` with static visualization functions
    - `plot_simulation_results(states: list[RobotState], model: BicycleModel)` - 2-column layout
    - Left: xy trajectory with rear/front wheel traces, start/end markers
    - Right: upper plot (steering angle vs time), lower plot (velocity vs time)
    - Uses model geometric methods for accurate wheel positioning
    - Added matplotlib as dev dependency
    - Reference: `temp/external/python-robotics/temp/mpl_visualizer.py:311-367`

* [feat_002] ✅ **COMPLETED** - Update `examples/01_basic_simulation.py` to demonstrate plotting
    - Import and call `plot_simulation_results(states, model)` after simulation
    - Graceful handling of missing matplotlib dependency

**Basic waypoint follower**

NOTE: "track" term is used for a series of waypoints that a robot must follow. This was previusly
often named "path", which has a conflicting meaning with filepath. Therefore useage of "track" is preferred.

This functionality should go into `src/rox_control/tracks.py`.
For reference, see `temp/external/python-robotics/examples/pure_pursuit/pure_pursuit_2.py`

* [feat_003] ✅ **COMPLETED** - `Track` class for waypoint management in `src/rox_control/tracks.py`
    - Port functionality from `Waypoints` class (see reference implementation)
    - Inherit from `UserList` for list-like behavior with indexing support
    - Use `rox_vectors.Vector` for waypoints with full type annotation coverage
    - **Essential methods:**
      - `find_next_idx(xy: Vector) -> int` - find next waypoint index (core tracking logic)
      - `target_reached` property - check if end of track reached
    - **Features:**
      - Automatic Vector conversion in `__init__` for tuples/lists
      - Private `_next_idx` attribute for internal state management
      - Validation: Ensure minimum 2 waypoints on creation
    - Added comprehensive unit tests with edge cases (20 test cases, 100% coverage)

* [feat_004] - Track generator functions in `src/tools/tracks.py`
    - **Function signature:** `generate_track(track_type: str, **kwargs) -> Track`
    - **Supported track types:**
      - `"square"` - Parameterized square path with `size: float = 1.0`, `resolution: int = 4`
      - `"circle"` - Circular path with `radius: float = 1.0`, `center: Vector = Vector(0,0)`, `resolution: int = 16`
      - `"figure8"` - Figure-8 pattern with `size: float = 1.0`, `resolution: int = 32`
    - **Features:**
      - Consistent waypoint spacing via optional `spacing: float` parameter
      - Parameter validation with clear error messages
      - Returns properly initialized `Track` objects
    - Add unit tests for all track types and parameter combinations

* [feat_005] - Pure pursuit controller in `src/rox_control/controllers.py` (new module)
    - **Class-based design:** `PurePursuitController` with clean stateful interface
    - **Constructor parameters:**
      - `look_ahead_distance: float = 0.2` - lookahead distance for target calculation
      - `velocity_vector_length: float = 0.1` - robot velocity projection length
      - `proportional_gain: float = 1.0` - steering control gain
      - `target_speed: float = 0.1` - desired robot velocity
    - **Core interface:**
      - `set_track(track: Track)` - assign track for controller to follow
      - `control(robot_state: RobotState) -> ControlOutput` - returns structured control data
    - **ControlOutput dataclass:**
      - `curvature: float` - steering command (essential)
      - `velocity: float` - speed command (essential)
      - `target_point: Vector` - target point for visualization
      - `future_position: Vector` - projected robot position
      - `angle_error: float` - pure pursuit angle error for debugging
      - `track_complete: bool` - whether track following is finished
    - **Features:**
      - Integrates with `Track` objects from feat_003 and `RobotState` from bicycle model
      - Extensible output format for visualization and debugging
      - Handles track completion and safety edge cases
    - Port core algorithm from `target_position()` and `proportional_control()` functions
    - Add comprehensive unit tests and integration tests with Track class

**Architectural Considerations:**
- **Module Organization:** Consider splitting `tracks.py` if it grows large - separate `Track`/generators from controller logic
- **Testing Strategy:** Each feature in core library needs comprehensive unit tests with edge cases (empty tracks, single waypoint, malformed data). Code contained in `tools` may have less strict testing, we should not care too much about edge cases here.
- **Type Safety:** Full mypy compliance with no `# type: ignore` comments required
- **Documentation:** Add docstring for each class/function where function name may need some clarification. A single line docstring is enough, typehints should provide enough context about the interface.



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

## Implementation Notes

### Completed Features

**Static Visualization (feat_001, feat_002)**
- Implementation: [`src/tools/plot.py`](src/tools/plot.py) - `plot_simulation_results()` function
- Example usage: [`examples/01_basic_simulation.py`](examples/01_basic_simulation.py)
- Tests: [`tests/test_smoke.py`](tests/test_smoke.py) - import validation

**Track Class (feat_003)**
- Implementation: [`src/rox_control/tracks.py`](src/rox_control/tracks.py) - `Track` class with UserList inheritance
- Tests: [`tests/test_tracks.py`](tests/test_tracks.py) - 20 test cases with 100% coverage
