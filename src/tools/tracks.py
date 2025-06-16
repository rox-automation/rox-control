#!/usr/bin/env python3
"""Track generator functions for creating common path patterns."""

import math
from typing import Any

from rox_control.tracks import Track
from rox_vectors import Vector


def generate_track(track_type: str, **kwargs: Any) -> Track:
    """Generate Track objects for common path patterns.

    Args:
        track_type: Type of track to generate ("square", "circle", "figure8")
        **kwargs: Track-specific parameters

    Returns:
        Track object with generated waypoints

    Raises:
        ValueError: If track_type is not supported or parameters are invalid
    """
    if track_type == "square":
        return _generate_square_track(**kwargs)
    elif track_type == "circle":
        return _generate_circle_track(**kwargs)
    elif track_type == "figure8":
        return _generate_figure8_track(**kwargs)
    else:
        raise ValueError(f"Unsupported track type: {track_type}")


def _generate_square_track(size: float = 1.0) -> Track:
    """Generate square track with parameterized dimensions.

    Args:
        size: Side length of the square

    Returns:
        Track with 5 waypoints (4 corners + closing point)

    Raises:
        ValueError: If size is invalid
    """
    if size <= 0:
        raise ValueError("Size must be positive")

    # Define square starting at origin (0,0) going clockwise
    waypoints = [
        Vector(0, 0),  # Start at origin
        Vector(size, 0),  # Right
        Vector(size, size),  # Top-right
        Vector(0, size),  # Top-left
        Vector(0, 0),  # Back to start
    ]

    return Track(waypoints)  # type: ignore[arg-type]


def _generate_circle_track(
    radius: float = 1.0,
    center: Vector | None = None,
    resolution: int = 16,
) -> Track:
    """Generate circular track with parameterized radius and center.

    Args:
        radius: Circle radius
        center: Center point of the circle
        resolution: Number of waypoints around the circle

    Returns:
        Track with circular waypoints

    Raises:
        ValueError: If parameters are invalid
    """
    if radius <= 0:
        raise ValueError("Radius must be positive")
    if resolution < 3:
        raise ValueError("Resolution must be at least 3")

    if center is None:
        center = Vector(0, 0)

    waypoints = []
    for i in range(resolution):
        angle = 2 * math.pi * i / resolution
        x = center.x + radius * math.cos(angle)
        y = center.y + radius * math.sin(angle)
        waypoints.append(Vector(x, y))

    return Track(waypoints)  # type: ignore[arg-type]


def _generate_figure8_track(size: float = 1.0, resolution: int = 32) -> Track:
    """Generate figure-8 track with parameterized size.

    Args:
        size: Overall size of the figure-8
        resolution: Number of waypoints for the complete figure-8

    Returns:
        Track with figure-8 waypoints

    Raises:
        ValueError: If parameters are invalid
    """
    if size <= 0:
        raise ValueError("Size must be positive")
    if resolution < 6:
        raise ValueError("Resolution must be at least 6")

    waypoints = []
    for i in range(resolution):
        t = 2 * math.pi * i / resolution
        # Parametric equations for figure-8 (lemniscate)
        x = size * math.sin(t) / (1 + math.cos(t) ** 2)
        y = size * math.sin(t) * math.cos(t) / (1 + math.cos(t) ** 2)
        waypoints.append(Vector(x, y))

    return Track(waypoints)  # type: ignore[arg-type]
