#!/usr/bin/env python3
"""
Pure Pursuit A controller example.

Demonstrates pure pursuit path tracking on a 20m square track using the
PurePursuitA controller with realistic parameters for larger scale operation.

Copyright (c) 2025 ROX Automation - Jev Kuznetsov
"""

import math
import time
from dataclasses import asdict

from rox_control.controllers import PurePursuitA
from rox_control.tools.bicicle_model import BicycleModel, RobotState
from rox_control.tools.plot import plot_simulation_data
from rox_control.tools.simulation import (
    SimulationData,
    SimulationState,
    present_results,
)
from rox_control.tools.tracks import rectangular_track

WHEELBASE = 5.0  # distance between front and rear wheels in meters

ANIMATE = True  # Set to False to disable animation


def run_pure_pursuit_simulation(
    controller: PurePursuitA, model: BicycleModel, dt: float = 0.01
) -> list[SimulationState]:
    """Run pure pursuit simulation with timeout safety and debug data collection."""

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

    # Create initial SimulationState with no debug data
    initial_state = model.state
    states = [SimulationState(**asdict(initial_state))]

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

        # Calculate projected path for debug visualization
        projected_path = None
        try:
            proj_x, proj_y = model.get_projected_path(distance=8.0, num_points=30)
            projected_path = list(zip(proj_x, proj_y, strict=False))
        except Exception:
            # If projected path calculation fails, continue without it
            pass

        # Create SimulationState with debug data
        simulation_state = SimulationState(
            **asdict(new_state),
            controller_output=control_output,
            projected_path=projected_path,
        )
        states.append(simulation_state)

    else:
        # Timeout occurred
        print(f"Simulation timeout after {timeout:.1f}s ({len(states)} steps)")
        print("Controller may have failed to reach target")

    return states


def main() -> None:
    """Run the pure pursuit controller example."""
    t_start = time.time()

    # Create 20m x 20m square track
    track = rectangular_track(L=50.0, B=10.0)  # 50m perimeter
    print(f"Generated square track with {len(track)} waypoints")

    # Create bicycle model with realistic parameters for larger scale
    model = BicycleModel(
        wheelbase=WHEELBASE,
        accel=3.0,  # 3 m/s² acceleration
        steering_speed=math.radians(60),  # 60°/s steering rate
        max_steering_angle=math.radians(45),  # 45° max steering
        max_velocity=15.0,  # 15 m/s max velocity
    )

    # Create pure pursuit controller with scaled parameters
    controller = PurePursuitA(
        look_ahead_distance=5.0,  # 2m lookahead (scaled from 0.2m)
        velocity_vector_length=1.0,  # 1m velocity vector (scaled from 0.1m)
        proportional_gain=1.0,  # Same gain as reference
        target_speed=2.0,
    )

    # Set track for controller
    controller.set_track(track)

    # Run simulation
    states = run_pure_pursuit_simulation(controller, model)

    t_end = time.time()

    # Convert SimulationState list to RobotState list for present_results (legacy compatibility)
    robot_states = [
        RobotState(
            **{
                k: v
                for k, v in asdict(state).items()
                if k in RobotState.__dataclass_fields__
            }
        )
        for state in states
    ]
    present_results(robot_states, t_end - t_start)

    # Generate animation for viewing results
    print("\nStarting animation...")
    try:
        simulation_data = SimulationData(states=states, track=track)

        # Plot using unified interface - same function call, just different animate parameter
        plot_simulation_data(
            data=simulation_data,
            animate=ANIMATE,
            animation_speed=1.0,
        )
        print("Animation completed successfully!")

    except ImportError:
        print("Matplotlib not available - skipping animation")
    except Exception as e:
        print(f"Error generating animation: {e}")


if __name__ == "__main__":
    main()
