"""
Tools subpackage for rox-control development and visualization.

This subpackage contains simulation, visualization, and utility tools for developing
and testing path tracking controllers. Some tools require additional dependencies
(like matplotlib) that are available via the 'tools' extra.

Install with visualization tools:
    pip install rox-control[tools]

Without visualization tools:
    pip install rox-control

Copyright (c) 2025 ROX Automation - Jev Kuznetsov
"""

from typing import Callable

# Always available tools (no extra dependencies)
from .bicicle_model import BicycleModel, RobotState
from .simulation import SimulationData, SimulationState, present_results
from .tracks import generate_track, rectangular_track

# Matplotlib-dependent tools with graceful fallback
try:
    import matplotlib  # noqa: F401

    from .plot import plot_simulation_data, plot_simulation_results

    _HAS_VISUALIZATION = True
except ImportError:
    _HAS_VISUALIZATION = False

    def _visualization_unavailable(name: str) -> Callable[..., None]:
        """Factory for creating unavailable visualization function stubs."""

        def _stub(*args: object, **kwargs: object) -> None:
            raise ImportError(
                f"Visualization function '{name}' requires matplotlib. "
                f"Install with: pip install rox-control[tools]"
            )

        return _stub

    # Create stub functions that provide helpful error messages
    plot_simulation_results = _visualization_unavailable("plot_simulation_results")
    plot_simulation_data = _visualization_unavailable("plot_simulation_data")

# Public API
__all__ = [
    # Utility flag
    "HAS_VISUALIZATION",
    # Simulation and modeling
    "BicycleModel",
    "RobotState",
    "SimulationData",
    "SimulationState",
    # Track generation
    "generate_track",
    "plot_simulation_data",
    # Visualization (may be stubs if matplotlib not available)
    "plot_simulation_results",
    "present_results",
    "rectangular_track",
]

# Utility for checking if visualization is available
HAS_VISUALIZATION = _HAS_VISUALIZATION
