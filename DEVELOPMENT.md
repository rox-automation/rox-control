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

* [feat_008] 🔄 **REFACTORING** - **Decouple visualization from simulation objects**. Current implementation has design issues that need addressing:

  **Current Issues:**
  - `SimulationData` class is in `plot.py` instead of `tools/simulation.py`
  - `create_simulation_data()` calculates controller outputs after simulation, violating decoupling
  - Animation interface is overcomplicated with projected path complexity
  - Examples have inconsistent plotting interfaces (should only differ by `animate=True`)

  **New Refactoring Plan:**
  - **Step 1**: Move `SimulationData` and related functions to `tools/simulation.py`
  - **Step 2**: Create `SimulationState` extending `RobotState` with optional debug data:
    ```python
    @dataclass(frozen=True)
    class SimulationState:
        # All RobotState fields
        x: float = 0.0
        y: float = 0.0
        theta: float = 0.0
        v: float = 0.0
        steering_angle: float = 0.0
        time: float = 0.0
        front_x: float = 0.0
        front_y: float = 0.0

        # Optional debug data (user responsibility during simulation)
        controller_output: ControlOutput | None = None
        projected_path: list[tuple[float, float]] | None = None
    ```
  - **Step 3**: Simplify `SimulationData` to just contain pre-computed states:
    ```python
    @dataclass(frozen=True)
    class SimulationData:
        states: list[SimulationState]
        track: Track | None = None
    ```
  - **Step 4**: Make both examples use identical interface: `plot_simulation_data(data, animate=False/True)`
  - **Step 5**: User populates debug data during simulation if needed (no post-calculation)

  **Benefits:**
  - True decoupling - all calculations during simulation phase
  - Consistent simple interface between static and animated plotting
  - User controls what debug data to include
  - No overcomplicated projected path handling in animation

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
