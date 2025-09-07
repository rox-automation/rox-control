# rox-control

Controllers for path tracking.

This repository provides pure python implementations of controllers for path tracking and basic simulation and visualisation tools.

The goal is to provide simple, yet production ready controllers that are easy to understand, integrate and maintain.

Why this repo?

1. Because [PythonRobotics](https://github.com/AtsushiSakai/PythonRobotics) (which is great btw) has a purely educational purpose.
2. Because ROS2 controllers (Nav2 etc.) are C++ centered and require a lot of complexity.

## What's in the repo

**rox-control** provides production-ready path tracking controllers for robotics applications:

- **Controllers**: `PurePursuitA` - Pure pursuit path tracking controller with lookahead-based steering
- **Track Management**: `Track` class for waypoint/path management and interpolation
- **Simulation Tools**: Bicycle kinematics model, visualization, and plotting utilities (optional `[tools]` extra)
- **Examples**: Ready-to-run demonstrations of controller usage

The package focuses on simplicity and production readiness, unlike educational implementations. Core controllers are packaged separately from simulation/visualization tools.

## Quick start

### Installation

Choose your installation based on your needs:

```bash
# Production use: Core controllers only (minimal dependencies)
pip install rox-control

# Development/Research: Include simulation and visualization tools  
pip install rox-control[tools]
```

For local development from source:

```bash
git clone <repository-url>
cd rox-control
invoke create-venv  # Creates venv and installs all dependencies
```

### Basic usage

```python
from rox_control import Track
from rox_control.controllers import PurePursuitA

# Create a track from waypoints
track = Track([(0, 0), (10, 0), (10, 10), (0, 10)])

# Set up controller
controller = PurePursuitA(
    look_ahead_distance=2.0,
    target_speed=5.0
)
controller.set_track(track)

# Use in control loop
control_output = controller.control(robot_state)
```

### Examples

Run the included examples to see the controllers in action (requires `[tools]` installation):

```bash
python examples/01_basic_simulation.py     # Basic bicycle model simulation
python examples/02_pure_pursuit_a.py       # Pure pursuit path tracking
```

## Package Structure

* **Core package**: `src/rox_control/` - Controllers and track management (always available)
* **Tools package**: `src/rox_control/tools/` - Simulation and visualization tools (optional `[tools]` extra)
* **Examples**: `examples/` - Ready-to-run demonstrations


## References

* [PythonRobotics](https://github.com/AtsushiSakai/PythonRobotics) - Python code collection and a textbook of robotics algorithms.
* [Robot Kinematics](https://control.ros.org/rolling/doc/ros2_controllers/doc/mobile_robot_kinematics.html) - an introduction to mobile robot kinematics and the nomenclature
* [Automatic Steering Methods for Path Tracking](docs/Automatic_Steering_Methods_for_Autonomous_Automobile_Path_Tracking.pdf)


## Development

see [DEVELOPMENT.md](DEVELOPMENT.md)
