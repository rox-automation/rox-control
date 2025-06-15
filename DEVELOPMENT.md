# Development

## Design philosophy

* Simulation and visualisation are decoupled.
* The system is synchronous, `.step(dt)` is used to calculate next step in time.
* Simulation runs as quickly as possible (no `sleep` delays) and produces a list of `RobotState`s.
* Visualisation uses simulation data to visualize it with plots or replay with liveplots, which use `sleep`.

## Feature roadmap

**Static Visualization Tools**

* [feat_001] Create `src/tools/plot.py` with static visualization functions
    - `plot_simulation_results(states: list[RobotState])` - 2-column layout
    - Left: xy trajectory with wheel position traces
    - Right: upper plot (steering angle vs time), lower plot (velocity vs time)
    - Reference: `temp/external/python-robotics/temp/mpl_visualizer.py:311-367`

* [feat_002] Update `examples/01_basic_simulation.py` to demonstrate plotting
    - Import and call `plot_simulation_results(states)` after simulation

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
* `tasks.py` - Automation tasks via invoke
* `pyproject.toml` - Modern Python packaging configuration
