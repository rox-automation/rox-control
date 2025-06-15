#!/usr/bin/env python3
"""
Animation debugging tool for controller analysis.

Provides step-by-step visualization of robot simulation with controller debug info
to help identify and fix path following issues.

Copyright (c) 2025 ROX Automation - Jev Kuznetsov
"""

import math
from dataclasses import dataclass
from typing import TYPE_CHECKING

import matplotlib.animation as animation
import matplotlib.pyplot as plt

from rox_control.tracks import Track

if TYPE_CHECKING:
    from rox_control.controllers import ControlOutput
    from tools.bicicle_model import RobotState


@dataclass
class SimulationFrame:
    """Single frame of simulation data for animation."""

    step: int
    robot_state: "RobotState"
    control_output: "ControlOutput"
    front_wheel_pos: tuple[float, float]


def animate_simulation(
    frames: list[SimulationFrame],
    track: Track,
    animation_speed: float = 1.0,
    show_debug_info: bool = True,
) -> None:
    """
    Animate simulation with step-by-step debugging visualization.

    Args:
        frames: List of simulation frames with robot state and controller data
        track: Track waypoints to visualize
        animation_speed: Playback speed multiplier (1.0 = real-time)
        show_debug_info: Whether to print debug info during animation
    """
    if not frames:
        print("No frames to animate")
        return

    # Set up the figure and axis
    fig, ax = plt.subplots(figsize=(12, 10))

    # Calculate bounds from track and trajectory
    all_x = [frame.robot_state.x for frame in frames]
    all_y = [frame.robot_state.y for frame in frames]
    track_x = [wp.x for wp in track.data]
    track_y = [wp.y for wp in track.data]

    all_x.extend(track_x)
    all_y.extend(track_y)

    x_range = max(all_x) - min(all_x)
    y_range = max(all_y) - min(all_y)
    padding = max(x_range, y_range) * 0.1 + 2.0

    ax.set_xlim(min(all_x) - padding, max(all_x) + padding)
    ax.set_ylim(min(all_y) - padding, max(all_y) + padding)
    ax.set_aspect("equal")
    ax.grid(True, alpha=0.3)
    ax.set_xlabel("X Position (m)")
    ax.set_ylabel("Y Position (m)")
    ax.set_title("Pure Pursuit Controller Debug Animation")

    # Plot static elements
    # Track waypoints and path
    ax.plot(track_x, track_y, "k-", linewidth=3, label="Track", alpha=0.7)
    ax.plot(track_x, track_y, "ko", markersize=6, alpha=0.7)

    # Robot trajectory (will be built up during animation)
    (traj_line,) = ax.plot([], [], "b-", linewidth=2, label="Robot Path", alpha=0.8)

    # Robot visualization elements
    (robot_dot,) = ax.plot([], [], "bo", markersize=8, label="Robot Position")
    (front_wheel_dot,) = ax.plot([], [], "ro", markersize=6, label="Front Wheel")
    heading_arrow = ax.annotate(
        "",
        xy=(0, 0),
        xytext=(0, 0),
        arrowprops={"arrowstyle": "->", "color": "blue", "lw": 2},
    )

    # Controller debug elements
    (target_dot,) = ax.plot([], [], "go", markersize=8, label="Target Point")
    (future_pos_dot,) = ax.plot([], [], "mo", markersize=6, label="Future Position")
    (lookahead_line,) = ax.plot(
        [], [], "g--", linewidth=2, alpha=0.7, label="Lookahead"
    )

    # Legend
    ax.legend(loc="upper right", bbox_to_anchor=(1.0, 1.0))

    # Text display for debug info
    info_text = ax.text(
        0.02,
        0.98,
        "",
        transform=ax.transAxes,
        verticalalignment="top",
        fontfamily="monospace",
        bbox={"boxstyle": "round", "facecolor": "wheat", "alpha": 0.8},
    )

    def update(frame_idx: int) -> tuple:
        """Update function for animation."""
        if frame_idx >= len(frames):
            return (
                traj_line,
                robot_dot,
                front_wheel_dot,
                target_dot,
                future_pos_dot,
                lookahead_line,
                info_text,
            )

        frame = frames[frame_idx]
        state = frame.robot_state
        control = frame.control_output
        front_pos = frame.front_wheel_pos

        # Update trajectory (accumulate path)
        traj_x = [f.robot_state.x for f in frames[: frame_idx + 1]]
        traj_y = [f.robot_state.y for f in frames[: frame_idx + 1]]
        traj_line.set_data(traj_x, traj_y)

        # Update robot position
        robot_dot.set_data([state.x], [state.y])
        front_wheel_dot.set_data([front_pos[0]], [front_pos[1]])

        # Update heading arrow
        arrow_length = 3.0
        arrow_end_x = state.x + arrow_length * math.cos(state.theta)
        arrow_end_y = state.y + arrow_length * math.sin(state.theta)
        heading_arrow.set_position((arrow_end_x, arrow_end_y))
        heading_arrow.xy = (state.x, state.y)

        # Update controller debug elements
        target_dot.set_data([control.target_point.x], [control.target_point.y])
        future_pos_dot.set_data(
            [control.future_position.x], [control.future_position.y]
        )

        # Lookahead line from robot to target
        lookahead_line.set_data(
            [state.x, control.target_point.x], [state.y, control.target_point.y]
        )

        # Update debug info text
        info_str = (
            f"Step: {frame.step:4d}  Time: {state.time:6.2f}s\n"
            f"Robot: ({state.x:6.2f}, {state.y:6.2f})  θ: {math.degrees(state.theta):6.1f}°\n"
            f"Speed: {state.v:5.2f} m/s  Steering: {math.degrees(state.steering_angle):6.1f}°\n"
            f"Target: ({control.target_point.x:6.2f}, {control.target_point.y:6.2f})\n"
            f"Curvature: {control.curvature:7.4f}  Angle Error: {math.degrees(control.angle_error):6.1f}°\n"
            f"Complete: {control.track_complete}"
        )
        info_text.set_text(info_str)

        # Print debug info to terminal if enabled
        if show_debug_info:
            print(
                f"Frame {frame_idx:3d}: "
                f"pos=({state.x:6.2f}, {state.y:6.2f}), "
                f"θ={math.degrees(state.theta):6.1f}°, "
                f"target=({control.target_point.x:6.2f}, {control.target_point.y:6.2f}), "
                f"curvature={control.curvature:7.4f}"
            )

        return (
            traj_line,
            robot_dot,
            front_wheel_dot,
            target_dot,
            future_pos_dot,
            lookahead_line,
            info_text,
        )

    # Calculate animation interval (milliseconds)
    # Each frame represents dt=0.01s, so interval = 10ms * animation_speed
    interval = max(10, int(10 / animation_speed))

    print(f"Starting animation with {len(frames)} frames")
    print(f"Animation speed: {animation_speed}x")
    print(f"Frame interval: {interval}ms")
    print("Debug info will be printed to terminal during playback")
    print()

    # Create and run animation (keep reference to prevent garbage collection)
    anim = animation.FuncAnimation(
        fig, update, frames=len(frames), interval=interval, blit=False, repeat=False
    )

    plt.tight_layout()
    plt.show()
    
    # Keep animation reference alive until show() completes
    _ = anim  # Prevent garbage collection
    print(f"\nAnimation complete!")
