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

* [feat_006] ✅ **COMPLETED** - Pure Pursuit A controller example in [`examples/02_pure_pursuit_a.py`](examples/02_pure_pursuit_a.py) - Complete simulation with 20m square track, timeout safety, and integrated animation debugging

* [feat_007] ✅ **COMPLETED** - Animation debugging tool unified in [`src/tools/plot.py`](src/tools/plot.py) - Real-time controller visualization with debug overlays, projected paths, and configurable animation speed

* [feat_008] - **Decouple visualization from simulation objects**. Current `plot_simulation_results()` function violates the design philosophy by requiring `BicycleModel` and `Controller` instances for runtime calculations during visualization.

  **Coupling Issues:**
  - Function calls `model.get_front_wheel_pos()` and `model.get_projected_path()` during rendering
  - Animation mode requires `controller.control()` calls for debug visualization
  - Model state is mutated during visualization (`model.state = state`)
  - Values are recalculated on every animation frame instead of being pre-computed

  **Proposed Solution:**
  - **Step 1**: Extend `RobotState` with front wheel position fields:
    - Add `front_x: float` and `front_y: float` to `RobotState` NamedTuple
    - Update `BicycleModel.step()` to calculate and include front wheel position in state
  - **Step 2**: Create `SimulationData` frozen dataclass containing all pre-computed visualization data:
    - `states: list[RobotState]` - Robot trajectory (now includes front wheel positions)
    - `track: Track | None` - Track waypoints
    - `controller_outputs: list[ControlOutput] | None` - Pre-computed controller states for debug
    - `projected_paths: list[list[tuple[float, float]]] | None` - Pre-calculated projected paths
  - **Step 3**: Refactor `plot_simulation_results()` to accept only `SimulationData` object
  - **Step 4**: Move all calculation logic to simulation phase, not visualization phase

  **Benefits:**
  - True decoupling of visualization from simulation objects
  - Faster animation (no runtime calculations)
  - Immutable visualization data
  - Easier testing of visualization independently

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
