#!/usr/bin/env python3
"""Track generator functions for creating common path patterns."""

import math
from typing import Any

from rox_vectors import Vector

from rox_control.tracks import Track


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


def _generate_square_track(
    size: float = 1.0, resolution: int = 4, spacing: float | None = None
) -> Track:
    """Generate square track with parameterized dimensions.

    Args:
        size: Side length of the square
        resolution: Number of waypoints per side (minimum 1)
        spacing: Optional uniform spacing between waypoints

    Returns:
        Track with square waypoints

    Raises:
        ValueError: If parameters are invalid
    """
    if size <= 0:
        raise ValueError("Size must be positive")
    if resolution < 1:
        raise ValueError("Resolution must be at least 1")

    # Define square corners
    half_size = size / 2
    corners = [
        Vector(-half_size, -half_size),  # Bottom-left
        Vector(half_size, -half_size),  # Bottom-right
        Vector(half_size, half_size),  # Top-right
        Vector(-half_size, half_size),  # Top-left
    ]

    waypoints = []
    for i in range(4):  # Four sides
        start = corners[i]
        end = corners[(i + 1) % 4]

        # Generate waypoints along this side
        for j in range(resolution):
            t = j / resolution
            point = Vector(
                start.x + t * (end.x - start.x), start.y + t * (end.y - start.y)
            )
            waypoints.append(point)

    # Apply spacing if specified
    if spacing is not None:
        waypoints = _apply_spacing(waypoints, spacing)

    return Track(waypoints)  # type: ignore[arg-type]


def _generate_circle_track(
    radius: float = 1.0,
    center: Vector | None = None,
    resolution: int = 16,
    spacing: float | None = None,
) -> Track:
    """Generate circular track with parameterized radius and center.

    Args:
        radius: Circle radius
        center: Center point of the circle
        resolution: Number of waypoints around the circle
        spacing: Optional uniform spacing between waypoints

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

    # Apply spacing if specified
    if spacing is not None:
        waypoints = _apply_spacing(waypoints, spacing)

    return Track(waypoints)  # type: ignore[arg-type]


def _generate_figure8_track(
    size: float = 1.0, resolution: int = 32, spacing: float | None = None
) -> Track:
    """Generate figure-8 track with parameterized size.

    Args:
        size: Overall size of the figure-8
        resolution: Number of waypoints for the complete figure-8
        spacing: Optional uniform spacing between waypoints

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

    # Apply spacing if specified
    if spacing is not None:
        waypoints = _apply_spacing(waypoints, spacing)

    return Track(waypoints)  # type: ignore[arg-type]


def _apply_spacing(waypoints: list[Vector], spacing: float) -> list[Vector]:
    """Apply consistent spacing between waypoints.

    Args:
        waypoints: Original waypoints
        spacing: Desired spacing between waypoints

    Returns:
        Waypoints with consistent spacing

    Raises:
        ValueError: If spacing is not positive
    """
    if spacing <= 0:
        raise ValueError("Spacing must be positive")

    if len(waypoints) < 2:
        return waypoints

    # Calculate total path length
    total_length = 0.0
    for i in range(len(waypoints)):
        next_i = (i + 1) % len(waypoints)
        total_length += abs(waypoints[next_i] - waypoints[i])

    # Calculate number of evenly spaced points
    num_points = max(2, int(total_length / spacing))

    # Generate evenly spaced waypoints
    spaced_waypoints = []
    target_distance = 0.0

    for i in range(num_points):
        target_distance = i * spacing

        # Find the segment containing the target distance
        cumulative_distance = 0.0
        for j in range(len(waypoints)):
            next_j = (j + 1) % len(waypoints)
            segment_length = abs(waypoints[next_j] - waypoints[j])

            if cumulative_distance + segment_length >= target_distance:
                # Interpolate within this segment
                t = (target_distance - cumulative_distance) / segment_length
                point = Vector(
                    waypoints[j].x + t * (waypoints[next_j].x - waypoints[j].x),
                    waypoints[j].y + t * (waypoints[next_j].y - waypoints[j].y),
                )
                spaced_waypoints.append(point)
                break

            cumulative_distance += segment_length

    return spaced_waypoints
