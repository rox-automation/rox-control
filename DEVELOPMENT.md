# Development

## Design philosophy

* Simulation and visualisation are decoupled.
* The system is synchronous, `.step(dt)` is used to calculate next step in time.
* Simulation runs as quickly as possible (no `sleep` delays) and produces a list of `RobotState`s.
* Visualisation uses simulation data to visualize it with plots or replay with liveplots, which use `sleep`.

## Feature roadmap

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

**Animation Tools**

* [feat_003] Create `src/tools/animation.py` with live animation functions
    - `animate_simulation(states: list[RobotState])` - replay with matplotlib animation
    - Progressive state replay with configurable playbook speed
    - Show projected path curve for front wheel during replay
    - Reference: `temp/external/python-robotics/temp/mpl_visualizer.py:369-443`

**Data Export & Replay**

* [feat_004] Add CSV export functionality to `examples/01_basic_simulation.py`
    - `save_states_to_csv(states: list[RobotState], filename: str)` helper function
    - Export time series data with proper headers (time, x, y, theta, v, steering_angle)

* [feat_005] Create `examples/02_replay_data.py`
    - Load CSV data and convert back to `RobotState` objects
    - Demonstrate static plotting and animation from saved data

**Advanced Visualization Features**

* [feat_006] Extend `src/tools/plot.py` with additional plot types
    - Trajectory analysis plots (curvature, acceleration profiles)
    - Multi-simulation comparison capabilities

* [feat_007] Extend `src/tools/animation.py` with interactive features
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
- API: `plot_simulation_results(states: list[RobotState], model: BicycleModel)`
- Uses the same `BicycleModel` instance that generated the states for consistency
- Front/rear wheel positions calculated using `model.get_front_wheel_pos()`
- Data extraction separated into `extract_trajectory_data()` helper function
- Matplotlib added as dev dependency only (visualization tooling)
- Example integration demonstrates graceful matplotlib dependency handling
- Smoke tests optimized for CI/CD speed (0.39s total test time)
