#!/usr/bin/env python3
"""
Pure Pursuit A controller example.

Demonstrates pure pursuit path tracking on a 20m square track using the
PurePursuitA controller with realistic parameters for larger scale operation.

Copyright (c) 2025 ROX Automation - Jev Kuznetsov
"""

import math
import time

from rox_control.controllers import PurePursuitA
from tools.bicicle_model import BicycleModel, RobotState
from tools.plot import create_simulation_data, plot_simulation_data
from tools.simulation import present_results
from tools.tracks import generate_track


def run_pure_pursuit_simulation(
    controller: PurePursuitA, model: BicycleModel, dt: float = 0.01
) -> list[RobotState]:
    """Run pure pursuit simulation with timeout safety."""

    # KISS timeout calculation: track_length / target_velocity * 5
    track_length = 80.0  # 20m square perimeter
    target_speed = controller.target_speed
    timeout = track_length / target_speed * 5
    max_steps = int(timeout / dt)

    print("Starting pure pursuit simulation:")
    print(f"  Track: 20m x 20m square ({track_length}m perimeter)")
    print(f"  Target speed: {target_speed} m/s")
    print(f"  Timeout: {timeout:.1f}s ({max_steps} steps)")
    print(f"  Time step: {dt}s")
    print()

    states = [model.state]

    for step in range(max_steps):
        # Get control command
        control_output = controller.control(model.state)

        # Check completion
        if control_output.track_complete:
            print(f"Target reached at step {step}")
            break

        # Apply control and step simulation
        model.set_control_command(control_output.curvature, control_output.velocity)
        new_state = model.step(dt)
        states.append(new_state)

        # Progress reporting (every 20 steps like reference)
        if step % 20 == 0 or step < 10:
            print(
                f"Step {step:3d}: pos=({new_state.x:5.2f}, {new_state.y:5.2f}), "
                f"heading={math.degrees(new_state.theta):6.1f}°, "
                f"target=({control_output.target_point.x:5.2f}, {control_output.target_point.y:5.2f})"
            )
    else:
        # Timeout occurred
        print(f"Simulation timeout after {timeout:.1f}s ({len(states)} steps)")
        print("Controller may have failed to reach target")

    return states


def main() -> None:
    """Run the pure pursuit controller example."""
    t_start = time.time()

    # Create 20m x 20m square track
    track = generate_track("square", size=20.0)
    print(f"Generated square track with {len(track)} waypoints")

    # Create bicycle model with realistic parameters for larger scale
    model = BicycleModel(
        wheelbase=2.5,  # 2.5m wheelbase (typical car)
        accel=3.0,  # 3 m/s² acceleration
        steering_speed=math.radians(60),  # 60°/s steering rate
        max_steering_angle=math.radians(45),  # 45° max steering
        max_velocity=15.0,  # 15 m/s max velocity
    )

    # Create pure pursuit controller with scaled parameters
    controller = PurePursuitA(
        look_ahead_distance=2.0,  # 2m lookahead (scaled from 0.2m)
        velocity_vector_length=1.0,  # 1m velocity vector (scaled from 0.1m)
        proportional_gain=1.0,  # Same gain as reference
        target_speed=5.0,  # 5 m/s target speed (scaled from 0.1 m/s)
    )

    # Set track for controller
    controller.set_track(track)

    # Run simulation
    states = run_pure_pursuit_simulation(controller, model)

    t_end = time.time()
    present_results(states, t_end - t_start)

    # Generate animation for debugging
    print("\nStarting debug animation...")
    try:
        # Create fresh track and controller for animation (to avoid state contamination)
        animation_track = generate_track("square", size=20.0)
        animation_controller = PurePursuitA(
            look_ahead_distance=2.0,
            velocity_vector_length=1.0,
            proportional_gain=1.0,
            target_speed=5.0,
        )
        animation_controller.set_track(animation_track)

        # Create simulation data with pre-computed controller outputs and projected paths (feat_008)
        simulation_data = create_simulation_data(
            states=states,
            track=animation_track,
            controller=animation_controller,
            model=model,
            include_projected_paths=True,
        )

        # Plot using new decoupled approach
        plot_simulation_data(
            data=simulation_data,
            animate=True,
            animation_speed=2.0,
            show_projected_path=True,
            show_debug_info=True,
        )
        print("Animation completed successfully!")

    except ImportError:
        print("Matplotlib not available - skipping animation")
    except Exception as e:
        print(f"Error generating animation: {e}")


if __name__ == "__main__":
    main()
