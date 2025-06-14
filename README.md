# rox-control

Controllers for path tracking.

This repository provides pure python implementations of controllers for path tracking and basic simulation and visualisation tools.

The goal is to provide simple, yet production ready controllers that are easy to understand, integrate and maintain.

Why this repo?

1. Because [PythonRobotics](https://github.com/AtsushiSakai/PythonRobotics) (which is great btw) has a purely educational purpose.
2. Because ROS2 controllers (Nav2 etc.) are C++ centered and require a lot of complexity.

## Repository structure

* main package is contained in `src/rox_control`
* supporting code (simulator, visualisation etc.) are in `src/tools`. These are not included in the package.


## References

* [PythonRobotics](https://github.com/AtsushiSakai/PythonRobotics) - Python code collection and a textbook of robotics algorithms.
* [Robot Kinematics](https://control.ros.org/rolling/doc/ros2_controllers/doc/mobile_robot_kinematics.html) - an introduction to mobile robot kinematics and the nomenclature
* [Automatic Steering Methods for Path Tracking](docs/Automatic_Steering_Methods_for_Autonomous_Automobile_Path_Tracking.pdf)


## Development

see [DEVELOPMENT.md](DEVELOPMENT.md)
