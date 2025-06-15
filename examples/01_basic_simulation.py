#!/usr/bin/env python3
"""
Demonstration of a basic bicycle kinematics simulation.

This example shows how to use the BicycleModel class to simulate
robot movement with realistic rate-limited steering and acceleration.

Copyright (c) 2025 ROX Automation - Jev Kuznetsov
"""

import math
import time
from dataclasses import dataclass

from tools.bycicle_model import BicycleModel, RobotState


@dataclass
class Maneuver:
    """Defines a single maneuver in the simulation"""

    name: str
    duration: float
    velocity: float
    steering_deg: float


# Define simulation maneuvers
MANEUVERS = [
    Maneuver("Drive straight", 5.0, velocity=5.0, steering_deg=0),
    Maneuver("Turn left", 3.0, velocity=5.0, steering_deg=20),
    Maneuver("Turn right", 3.0, velocity=5.0, steering_deg=-20),
    Maneuver("Stop", 3.0, velocity=0.0, steering_deg=0),
]

SIMULATION_DT = 0.01  # 10ms time step for smooth simulation


def run_simulation(model: BicycleModel) -> list[RobotState]:
    """Run the complete simulation sequence."""

    def run_maneuver(
        model: BicycleModel,
        maneuver: Maneuver,
        dt: float,
    ) -> list[RobotState]:
        """Run a single maneuver for given duration."""
        model.set_target_velocity(maneuver.velocity)
        model.set_target_steering_angle(math.radians(maneuver.steering_deg))

        states = []
        for _ in range(int(maneuver.duration / dt)):
            states.append(model.step(dt))
        return states

    all_states = [model.state]  # Start with initial state

    print("Running simulation maneuvers:")
    for maneuver in MANEUVERS:
        print(f"  {maneuver.name}...")

        maneuver_states = run_maneuver(
            model=model,
            maneuver=maneuver,
            dt=SIMULATION_DT,
        )
        all_states.extend(maneuver_states)

    return all_states


def present_results(states: list[RobotState], execution_time: float) -> None:
    """Present simulation results in a formatted way."""

    def print_states_table(states: list[RobotState], subsample: int = 50) -> None:
        """Print states in a nicely formatted table."""
        print(
            f"{'Time':>6} {'X':>8} {'Y':>8} {'Theta':>8} {'Velocity':>8} {'Steering':>8}"
        )
        print(
            f"{'(s)':>6} {'(m)':>8} {'(m)':>8} {'(deg)':>8} {'(m/s)':>8} {'(deg)':>8}"
        )
        print("-" * 54)

        for i, state in enumerate(states):
            if i % subsample == 0:
                print(
                    f"{state.time:6.1f} {state.x:8.2f} {state.y:8.2f} "
                    f"{math.degrees(state.theta):8.1f} {state.v:8.2f} "
                    f"{math.degrees(state.steering_angle):8.1f}"
                )

    print("\nBicycle Model Simulation Results")
    print("=" * 54)
    print_states_table(states)

    final_state = states[-1]
    print(f"\nSimulation completed in {execution_time:.3f} seconds")
    print(f"Final position: ({final_state.x:.2f}, {final_state.y:.2f})")
    print(f"Total simulation time: {final_state.time:.2f} seconds")
    print(f"States generated: {len(states)}")


def main() -> None:
    """Run the basic bicycle simulation example."""
    t_start = time.time()

    # Create a bicycle model with realistic parameters
    model = BicycleModel(
        wheelbase=2.5,  # 2.5m wheelbase (typical car)
        accel=2.0,  # 2 m/s² max acceleration
        steering_speed=math.radians(30),  # 30°/s steering rate
        max_steering_angle=math.radians(45),  # 45° max steering
        max_velocity=15.0,  # 15 m/s max velocity
    )

    states = run_simulation(model)

    t_end = time.time()
    present_results(states, t_end - t_start)

    # Demonstrate plotting functionality (feat_001)
    print("Generating plots...")
    try:
        import matplotlib.pyplot as plt

        from tools.plot import plot_simulation_results

        plot_simulation_results(states, model)
        plt.show()
        print("Plots displayed successfully!")

    except ImportError:
        print("Matplotlib not available - skipping plot generation")
    except Exception as e:
        print(f"Error generating plots: {e}")


if __name__ == "__main__":
    main()
