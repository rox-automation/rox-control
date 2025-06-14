#!/usr/bin/env python3
"""
Demonstration of a basic bicycle kinematics simulation.

This example shows how to use the BicycleModel class to simulate
robot movement with realistic rate-limited steering and acceleration.

Copyright (c) 2025 ROX Automation - Jev Kuznetsov
"""

import math
import time

from tools.bycicle_model import BicycleModel, RobotState


def print_states_table(states: list[RobotState], subsample: int = 50) -> None:
    """Print states in a nicely formatted table"""
    print(f"{'Time':>6} {'X':>8} {'Y':>8} {'Theta':>8} {'Velocity':>8} {'Steering':>8}")
    print(f"{'(s)':>6} {'(m)':>8} {'(m)':>8} {'(deg)':>8} {'(m/s)':>8} {'(deg)':>8}")
    print("-" * 54)

    for i, state in enumerate(states):
        if i % subsample == 0:  # Print every 10th state (every 1 second)
            print(
                f"{state.time:6.1f} {state.x:8.2f} {state.y:8.2f} "
                f"{math.degrees(state.theta):8.1f} {state.v:8.2f} "
                f"{math.degrees(state.steering_angle):8.1f}"
            )


def main() -> None:
    """Run the basic bicycle simulation example"""

    t_start = time.time()  # Record start time

    # Create a bicycle model with realistic parameters
    model = BicycleModel(
        wheelbase=2.5,  # 2.5m wheelbase (typical car)
        accel=2.0,  # 2 m/s² max acceleration
        steering_speed=math.radians(30),  # 30°/s steering rate
        max_steering_angle=math.radians(45),  # 45° max steering
        max_velocity=15.0,  # 15 m/s max velocity
    )

    states: list[RobotState] = [model.state]  # Start with initial state

    dt = 0.01  # 100 ms time step

    # Set target velocity
    model.set_target_velocity(5.0)  # 5 m/s

    # Drive straight for 5 seconds
    for _ in range(int(5 / dt)):
        states.append(model.step(dt))

    # Turn left for 3 seconds
    model.set_target_steering_angle(math.radians(20))  # 20 degrees left
    for _ in range(int(3 / dt)):
        states.append(model.step(dt))

    # Turn right for 3 seconds
    model.set_target_steering_angle(math.radians(-20))  # 20 degrees right
    for _ in range(int(3 / dt)):
        states.append(model.step(dt))

    # Go straight and decelerate to stop
    model.set_target_steering_angle(0.0)  # Straight steering
    model.set_target_velocity(0.0)  # Stop
    for _ in range(int(3 / dt)):
        states.append(model.step(dt))

    t_end = time.time()  # Record end time
    # Print results
    print("Bicycle Model Simulation Results")
    print("=" * 54)
    print_states_table(states)

    print(f"\nSimulation completed in {t_end - t_start:.3f} seconds")
    print(f"Final position: ({states[-1].x:.2f}, {states[-1].y:.2f})")
    print(f"Total sim time {states[-1].time:.2f} seconds")


if __name__ == "__main__":
    main()
