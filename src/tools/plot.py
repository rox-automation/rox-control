#!/usr/bin/env python3
"""
Static visualization functions for bicycle simulation results.

This module provides plotting functionality for visualizing simulation results
from the BicycleModel. The main function creates a 2-panel layout showing
trajectory and time series data.

Copyright (c) 2025 ROX Automation - Jev Kuznetsov
"""

import math
from typing import Any

import matplotlib.pyplot as plt

from .bycicle_model import BicycleModel, RobotState


def extract_trajectory_data(
    states: list[RobotState], model: BicycleModel
) -> dict[str, Any]:
    """
    Extract trajectory and time series data from simulation states.

    Args:
        states: List of robot states from simulation
        model: BicycleModel instance used for the simulation

    Returns:
        Dictionary containing:
        - rear_trajectory: List of (x, y) tuples for rear wheel
        - front_trajectory: List of (x, y) tuples for front wheel
        - times: List of time values
        - steering_angles: List of steering angles in degrees
        - velocities: List of velocity values
    """
    if not states:
        return {
            "rear_trajectory": [],
            "front_trajectory": [],
            "times": [],
            "steering_angles": [],
            "velocities": [],
        }

    # Extract rear wheel trajectory (state position)
    rear_trajectory = [(state.x, state.y) for state in states]

    # Extract front wheel trajectory using model geometry
    front_trajectory = []
    for state in states:
        model.state = state  # Temporarily set state for geometry calculation
        front_trajectory.append(model.get_front_wheel_pos())

    # Extract time series data
    times = [state.time for state in states]
    steering_angles = [math.degrees(state.steering_angle) for state in states]
    velocities = [state.v for state in states]

    return {
        "rear_trajectory": rear_trajectory,
        "front_trajectory": front_trajectory,
        "times": times,
        "steering_angles": steering_angles,
        "velocities": velocities,
    }


def plot_simulation_results(states: list[RobotState], model: BicycleModel) -> None:
    """
    Plot simulation results showing trajectory and time series data.

    Creates a 2-column layout:
    - Left: xy trajectory with rear and front wheel traces
    - Right: upper plot (steering angle vs time), lower plot (velocity vs time)

    Args:
        states: List of robot states from simulation
        model: BicycleModel instance used for the simulation
    """
    if not states:
        print("Warning: No states provided for plotting")
        return

    # Extract data for plotting
    data = extract_trajectory_data(states, model)

    # Create figure with 2-column layout
    fig, (ax_traj, ax_time_container) = plt.subplots(1, 2, figsize=(15, 6))

    # Left panel: Trajectory plot
    _plot_trajectory(ax_traj, data)

    # Right panel: Time series plots (2 stacked subplots)
    _plot_time_series(ax_time_container, data, fig)

    # Overall styling
    fig.suptitle("Bicycle Model Simulation Results", fontsize=14, fontweight="bold")
    plt.tight_layout()


def _plot_trajectory(ax: plt.Axes, data: dict[str, Any]) -> None:
    """Plot trajectory with rear and front wheel traces."""
    if not data["rear_trajectory"]:
        return

    # Extract coordinates
    rear_x, rear_y = zip(*data["rear_trajectory"], strict=False)
    front_x, front_y = zip(*data["front_trajectory"], strict=False)

    # Plot trajectories
    ax.plot(rear_x, rear_y, "b-", linewidth=2, label="Rear Wheel", alpha=0.8)
    ax.plot(front_x, front_y, "r--", linewidth=2, label="Front Wheel", alpha=0.8)

    # Mark start and end points
    ax.plot(rear_x[0], rear_y[0], "go", markersize=8, label="Start")
    ax.plot(rear_x[-1], rear_y[-1], "ro", markersize=8, label="End")

    # Calculate bounds with padding
    all_x = list(rear_x) + list(front_x)
    all_y = list(rear_y) + list(front_y)
    x_range = max(all_x) - min(all_x)
    y_range = max(all_y) - min(all_y)
    padding = max(x_range, y_range) * 0.1 + 1.0  # Minimum 1m padding

    ax.set_xlim(min(all_x) - padding, max(all_x) + padding)
    ax.set_ylim(min(all_y) - padding, max(all_y) + padding)

    # Styling
    ax.set_aspect("equal")
    ax.grid(True, alpha=0.3)
    ax.set_xlabel("X Position (m)")
    ax.set_ylabel("Y Position (m)")
    ax.set_title("Vehicle Trajectory")
    ax.legend(loc="best")


def _plot_time_series(
    ax_container: plt.Axes, data: dict[str, Any], fig: plt.Figure
) -> None:
    """Plot time series data in 2 stacked subplots."""
    # Remove the container axis and create 2 stacked subplots in its place
    ax_container.remove()

    # Create 2 stacked subplots in the right panel
    gs = fig.add_gridspec(2, 2, width_ratios=[1, 1], height_ratios=[1, 1])
    ax_steering = fig.add_subplot(gs[0, 1])
    ax_velocity = fig.add_subplot(gs[1, 1])

    times = data["times"]

    if not times:
        return

    # Upper plot: Steering angle vs time
    ax_steering.plot(times, data["steering_angles"], "g-", linewidth=2)
    ax_steering.grid(True, alpha=0.3)
    ax_steering.set_ylabel("Steering Angle (°)")
    ax_steering.set_title("Steering Dynamics")

    # Lower plot: Velocity vs time
    ax_velocity.plot(times, data["velocities"], "m-", linewidth=2)
    ax_velocity.grid(True, alpha=0.3)
    ax_velocity.set_xlabel("Time (s)")
    ax_velocity.set_ylabel("Velocity (m/s)")
    ax_velocity.set_title("Speed Profile")

    # Share x-axis for time plots
    ax_steering.sharex(ax_velocity)
    ax_steering.tick_params(labelbottom=False)  # Hide x-tick labels on upper plot
