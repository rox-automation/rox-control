#!/usr/bin/env python3
"""
Simulation utility functions for robotics examples.

Copyright (c) 2025 ROX Automation - Jev Kuznetsov
"""

import math
from dataclasses import dataclass
from typing import TYPE_CHECKING

from rox_control.tools.bicicle_model import RobotState
from rox_control.track import Track

if TYPE_CHECKING:  # pragma: no cover
    from rox_control.controllers.pure_pursuit_a import ControlOutput


@dataclass(frozen=True)
class SimulationState(RobotState):
    """Extended robot state with optional debug data for visualization."""

    controller_output: "ControlOutput | None" = None
    projected_path: list[tuple[float, float]] | None = None


@dataclass(frozen=True)
class SimulationData:
    """Pre-computed simulation data for visualization."""

    states: list[SimulationState]
    track: Track | None = None


def present_results(states: list["RobotState"], execution_time: float) -> None:
    """Present simulation results in a formatted way."""

    def print_states_table(states: list["RobotState"], nr_rows: int = 10) -> None:
        """Print states in a nicely formatted table."""
        print(
            f"{'Time':>6} {'X':>8} {'Y':>8} {'Theta':>8} {'Velocity':>8} {'Steering':>8}"
        )
        print(
            f"{'(s)':>6} {'(m)':>8} {'(m)':>8} {'(deg)':>8} {'(m/s)':>8} {'(deg)':>8}"
        )
        print("-" * 54)

        # Calculate subsample automatically based on desired number of rows
        subsample = max(1, len(states) // nr_rows)

        for i, state in enumerate(states):
            if i % subsample == 0:
                print(
                    f"{state.time:6.1f} {state.x:8.2f} {state.y:8.2f} "
                    f"{math.degrees(state.theta):8.1f} {state.v:8.2f} "
                    f"{math.degrees(state.steering_angle):8.1f}"
                )

    print("\nSimulation Results")
    print("=" * 54)
    print_states_table(states)

    final_state = states[-1]
    print(f"\nSimulation completed in {execution_time:.3f} seconds")
    print(f"Final position: ({final_state.x:.2f}, {final_state.y:.2f})")
    print(f"Total simulation time: {final_state.time:.2f} seconds")
    print(f"States generated: {len(states)}")
