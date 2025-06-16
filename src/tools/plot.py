#!/usr/bin/env python3
"""
Static visualization functions for bicycle simulation results.

This module provides plotting functionality for visualizing simulation results
from the BicycleModel. The main function creates a 2-panel layout showing
trajectory and time series data.

Copyright (c) 2025 ROX Automation - Jev Kuznetsov
"""

import math
from typing import TYPE_CHECKING, Any

import matplotlib.animation as animation
import matplotlib.pyplot as plt
from rox_control.tracks import Track

from .bicicle_model import BicycleModel, RobotState

if TYPE_CHECKING:
    from rox_control.controllers.pure_pursuit_a import Controller


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


def plot_simulation_results(
    states: list[RobotState],
    model: BicycleModel,
    track: Track | None = None,
    animate: bool = False,
    controller: "Controller | None" = None,
    animation_speed: float = 1.0,
    show_projected_path: bool = True,
    show_debug_info: bool = False,
    frame_skip: int | None = None,
) -> None:
    """
    Plot simulation results showing trajectory and time series data.

    Creates a 2-column layout:
    - Left: xy trajectory with rear and front wheel traces and track waypoints
    - Right: upper plot (steering angle vs time), lower plot (velocity vs time)

    Args:
        states: List of robot states from simulation
        model: BicycleModel instance used for the simulation
        track: Optional track to visualize waypoints in black
        animate: If True, create animated plot instead of static
        controller: Controller instance (required if animate=True and debug needed)
        animation_speed: Playback speed multiplier (1.0 = real-time)
        show_projected_path: Whether to show projected path in animation
        show_debug_info: Whether to show debug info during animation
        frame_skip: Skip frames for faster animation (auto-calculated if None)
    """
    if not states:
        print("Warning: No states provided for plotting")
        return

    if animate:
        _plot_animated_results(
            states,
            model,
            track,
            controller,
            animation_speed,
            show_projected_path,
            show_debug_info,
            frame_skip,
        )
    else:
        _plot_static_results(states, model, track)


def _plot_static_results(
    states: list[RobotState], model: BicycleModel, track: Track | None = None
) -> None:
    """Plot static simulation results."""
    # Extract data for plotting
    data = extract_trajectory_data(states, model)

    # Create figure with 2-column layout
    fig, (ax_traj, ax_time_container) = plt.subplots(1, 2, figsize=(15, 6))

    # Left panel: Trajectory plot
    _plot_trajectory(ax_traj, data, track)

    # Right panel: Time series plots (2 stacked subplots)
    _plot_time_series(ax_time_container, data, fig)

    # Overall styling
    fig.suptitle("Bicycle Model Simulation Results", fontsize=14, fontweight="bold")
    plt.tight_layout()
    plt.show()


def _plot_animated_results(
    states: list[RobotState],
    model: BicycleModel,
    track: Track | None = None,
    controller: "Controller | None" = None,
    animation_speed: float = 1.0,
    show_projected_path: bool = True,
    show_debug_info: bool = False,
    frame_skip: int | None = None,
) -> None:
    """Plot animated simulation results."""
    if not states:
        print("No states to animate")
        return

    # Auto-calculate frame skip for reasonable animation speed
    if frame_skip is None:
        # Target ~50-100 frames for smooth animation regardless of data density
        target_frames = 75
        frame_skip = max(1, len(states) // target_frames)

    # Create subsampled states for animation
    animation_states = states[::frame_skip]
    if states[-1] not in animation_states:  # Ensure we include the final state
        animation_states.append(states[-1])

    # Create figure with same 2-column layout as static
    fig, (ax_traj, ax_time_container) = plt.subplots(1, 2, figsize=(15, 6))

    # Remove the time container and create 2 stacked subplots
    ax_time_container.remove()
    gs = fig.add_gridspec(2, 2, width_ratios=[1, 1], height_ratios=[1, 1])
    ax_steering = fig.add_subplot(gs[0, 1])
    ax_velocity = fig.add_subplot(gs[1, 1])

    # Setup trajectory plot using existing function (with empty data initially)
    empty_data: dict[str, Any] = {
        "rear_trajectory": [],
        "front_trajectory": [],
        "times": [],
        "steering_angles": [],
        "velocities": [],
    }
    _plot_trajectory(ax_traj, empty_data, track)

    # Calculate bounds for consistent axis limits
    all_x = [state.x for state in states]
    all_y = [state.y for state in states]

    if track is not None:
        track_x = [wp.x for wp in track.data]
        track_y = [wp.y for wp in track.data]
        all_x.extend(track_x)
        all_y.extend(track_y)

    x_range = max(all_x) - min(all_x)
    y_range = max(all_y) - min(all_y)
    padding = max(x_range, y_range) * 0.1 + 2.0

    ax_traj.set_xlim(min(all_x) - padding, max(all_x) + padding)
    ax_traj.set_ylim(min(all_y) - padding, max(all_y) + padding)

    # Initialize dynamic trajectory elements
    (rear_trace,) = ax_traj.plot(
        [], [], "b-", linewidth=2, label="Rear Wheel", alpha=0.8
    )
    (front_trace,) = ax_traj.plot(
        [], [], "r--", linewidth=2, label="Front Wheel", alpha=0.8
    )
    (robot_rear_dot,) = ax_traj.plot([], [], "bo", markersize=8, label="Robot Position")
    (robot_front_dot,) = ax_traj.plot([], [], "ro", markersize=6, label="Front Wheel")

    # Controller debug elements (if controller provided)
    target_dot = None
    lookahead_line = None
    if controller is not None:
        (target_dot,) = ax_traj.plot([], [], "mo", markersize=8, label="Target Point")
        (lookahead_line,) = ax_traj.plot(
            [], [], "m--", linewidth=2, alpha=0.7, label="Lookahead"
        )

    # Projected path
    projected_path_line = None
    if show_projected_path:
        (projected_path_line,) = ax_traj.plot(
            [], [], "c:", linewidth=2, alpha=0.8, label="Projected Path"
        )

    # Heading arrow
    heading_arrow = ax_traj.annotate(
        "",
        xy=(0, 0),
        xytext=(0, 0),
        arrowprops={"arrowstyle": "->", "color": "blue", "lw": 2},
    )

    ax_traj.legend(loc="best")

    # Setup time series plots using existing function patterns
    ax_steering.set_xlim(0, 10)  # Rolling window
    ax_steering.set_ylim(-50, 50)  # Degrees
    ax_steering.grid(True, alpha=0.3)
    ax_steering.set_ylabel("Steering Angle (°)")
    ax_steering.set_title("Steering Dynamics")
    ax_steering.tick_params(labelbottom=False)

    ax_velocity.set_xlim(0, 10)  # Rolling window
    ax_velocity.set_ylim(0, 20)  # m/s
    ax_velocity.grid(True, alpha=0.3)
    ax_velocity.set_xlabel("Time (s)")
    ax_velocity.set_ylabel("Velocity (m/s)")
    ax_velocity.set_title("Speed Profile")

    (steering_line,) = ax_steering.plot([], [], "g-", linewidth=2)
    (velocity_line,) = ax_velocity.plot([], [], "m-", linewidth=2)

    ax_steering.sharex(ax_velocity)

    # Debug info text
    info_text = ax_traj.text(
        0.02,
        0.98,
        "",
        transform=ax_traj.transAxes,
        verticalalignment="top",
        fontfamily="monospace",
        bbox={"boxstyle": "round", "facecolor": "wheat", "alpha": 0.8},
    )

    def update(frame_idx: int) -> tuple:
        """Update function for animation."""
        if frame_idx >= len(animation_states):
            return ()

        current_state = animation_states[frame_idx]

        # Find corresponding index in original states for trajectory building
        original_idx = min(frame_idx * frame_skip, len(states) - 1)

        # Update trajectory traces (accumulate path up to current position)
        rear_x = [s.x for s in states[: original_idx + 1]]
        rear_y = [s.y for s in states[: original_idx + 1]]

        # Calculate front wheel positions
        front_positions = []
        for state in states[: original_idx + 1]:
            model.state = state
            front_positions.append(model.get_front_wheel_pos())
        front_x = [pos[0] for pos in front_positions]
        front_y = [pos[1] for pos in front_positions]

        rear_trace.set_data(rear_x, rear_y)
        front_trace.set_data(front_x, front_y)

        # Update robot positions
        model.state = current_state
        front_pos = model.get_front_wheel_pos()
        robot_rear_dot.set_data([current_state.x], [current_state.y])
        robot_front_dot.set_data([front_pos[0]], [front_pos[1]])

        # Update heading arrow
        arrow_length = 3.0
        arrow_end_x = current_state.x + arrow_length * math.cos(current_state.theta)
        arrow_end_y = current_state.y + arrow_length * math.sin(current_state.theta)
        heading_arrow.set_position((arrow_end_x, arrow_end_y))
        heading_arrow.xy = (current_state.x, current_state.y)

        # Get controller output once for all visualizations
        control_output = None
        if controller is not None:
            control_output = controller.control(current_state)

        # Update controller debug elements (if controller available and track not complete)
        if (
            control_output is not None
            and target_dot is not None
            and lookahead_line is not None
        ):
            # Only show target point and lookahead if track is still active
            if not control_output.track_complete:
                target_dot.set_data(
                    [control_output.target_point.x], [control_output.target_point.y]
                )
                lookahead_line.set_data(
                    [current_state.x, control_output.target_point.x],
                    [current_state.y, control_output.target_point.y],
                )
            else:
                # Hide target point and lookahead when track is complete
                target_dot.set_data([], [])
                lookahead_line.set_data([], [])

        # Update projected path
        if show_projected_path and projected_path_line is not None:
            proj_x, proj_y = model.get_projected_path(distance=8.0, num_points=30)
            projected_path_line.set_data(proj_x, proj_y)

        # Update time series data
        times = [s.time for s in states[: original_idx + 1]]
        steering_angles = [
            math.degrees(s.steering_angle) for s in states[: original_idx + 1]
        ]
        velocities = [s.v for s in states[: original_idx + 1]]

        if times:
            steering_line.set_data(times, steering_angles)
            velocity_line.set_data(times, velocities)

            # Update rolling window
            current_time = current_state.time
            window_size = 10.0
            ax_steering.set_xlim(max(0, current_time - window_size), current_time + 1)
            ax_velocity.set_xlim(max(0, current_time - window_size), current_time + 1)

        # Update debug info text
        debug_str = f"Step: {frame_idx:4d}  Time: {current_state.time:6.2f}s\n"
        debug_str += f"Robot: ({current_state.x:6.2f}, {current_state.y:6.2f})  θ: {math.degrees(current_state.theta):6.1f}°\n"
        debug_str += f"Speed: {current_state.v:5.2f} m/s  Steering: {math.degrees(current_state.steering_angle):6.1f}°"

        if control_output is not None:
            if not control_output.track_complete:
                debug_str += f"\nTarget: ({control_output.target_point.x:6.2f}, {control_output.target_point.y:6.2f})\n"
                debug_str += f"Curvature: {control_output.curvature:7.4f}  Active: True"
            else:
                debug_str += "\nTrack Complete: True"

        info_text.set_text(debug_str)

        # Print debug info if enabled
        if show_debug_info:
            print(
                f"Frame {frame_idx:3d}: pos=({current_state.x:6.2f}, {current_state.y:6.2f}), θ={math.degrees(current_state.theta):6.1f}°"
            )

        return ()

    # Animation setup
    interval = max(10, int(10 / animation_speed))

    print(
        f"Starting animation with {len(animation_states)} frames (skipping every {frame_skip} from {len(states)} total)"
    )
    print(f"Animation speed: {animation_speed}x, interval: {interval}ms")

    # Create and run animation
    fig.suptitle(
        "Bicycle Model Simulation Results (Animated)", fontsize=14, fontweight="bold"
    )
    plt.tight_layout()

    anim = animation.FuncAnimation(
        fig,
        update,
        frames=len(animation_states),
        interval=interval,
        blit=False,
        repeat=False,
    )

    plt.show()
    _ = anim  # Prevent garbage collection
    print("\nAnimation complete!")


def _plot_trajectory(
    ax: plt.Axes, data: dict[str, Any], track: Track | None = None
) -> None:
    """Plot trajectory with rear and front wheel traces and optional track."""
    if not data["rear_trajectory"]:
        # For animation setup, just configure the axis
        if track is not None:
            track_x = [waypoint.x for waypoint in track.data]
            track_y = [waypoint.y for waypoint in track.data]
            ax.plot(track_x, track_y, "k-", linewidth=3, label="Track", alpha=0.7)
            ax.plot(track_x, track_y, "ko", markersize=6, alpha=0.7)

        ax.set_aspect("equal")
        ax.grid(True, alpha=0.3)
        ax.set_xlabel("X Position (m)")
        ax.set_ylabel("Y Position (m)")
        ax.set_title("Vehicle Trajectory")
        return

    # Extract coordinates for static plotting
    rear_x, rear_y = zip(*data["rear_trajectory"], strict=False)
    front_x, front_y = zip(*data["front_trajectory"], strict=False)

    # Plot track waypoints first (so they appear behind the trajectory)
    if track is not None:
        track_x = [waypoint.x for waypoint in track.data]
        track_y = [waypoint.y for waypoint in track.data]
        ax.plot(track_x, track_y, "k-", linewidth=3, label="Track", alpha=0.7)
        ax.plot(track_x, track_y, "ko", markersize=6, alpha=0.7)

    # Plot trajectories
    ax.plot(rear_x, rear_y, "b-", linewidth=2, label="Rear Wheel", alpha=0.8)
    ax.plot(front_x, front_y, "r--", linewidth=2, label="Front Wheel", alpha=0.8)

    # Mark start and end points
    ax.plot(rear_x[0], rear_y[0], "go", markersize=8, label="Start")
    ax.plot(rear_x[-1], rear_y[-1], "ro", markersize=8, label="End")

    # Calculate bounds with padding (include track waypoints if present)
    all_x = list(rear_x) + list(front_x)
    all_y = list(rear_y) + list(front_y)

    if track is not None:
        track_x = [waypoint.x for waypoint in track.data]
        track_y = [waypoint.y for waypoint in track.data]
        all_x.extend(track_x)
        all_y.extend(track_y)
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
