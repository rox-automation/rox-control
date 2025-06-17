#!/usr/bin/env python3
"""Track class for waypoint management in robotics path following."""

from collections import UserList
from typing import Sequence

from rox_vectors import Vector


class Track(UserList):
    """Navigation waypoints collection with path tracking functionality."""

    def __init__(self, waypoints: Sequence[Vector | tuple[float, float]]):
        """Initialize Track with waypoints as Vector objects or (x, y) tuples."""
        # Convert all waypoints to Vector objects
        converted_waypoints = [
            Vector(*wp) if isinstance(wp, (tuple, list)) else wp for wp in waypoints
        ]

        # Validate minimum waypoint requirement
        if len(converted_waypoints) < 2:
            raise ValueError("Track must contain at least 2 waypoints")

        super().__init__(converted_waypoints)

    def find_closest_segment(self, robot_xy: Vector) -> tuple[int, Vector, float]:
        """Find closest track segment and project robot position onto it."""
        if len(self.data) < 2:
            raise ValueError("Track must have at least 2 waypoints")

        min_distance = float("inf")
        closest_segment_idx = 0
        closest_projected_point = Vector(0, 0)
        closest_distance_along = 0.0

        # Check each segment
        for i in range(len(self.data) - 1):
            waypoint_a = self.data[i]
            waypoint_b = self.data[i + 1]

            # Project robot position onto line segment
            from rox_vectors.vectors import point_on_line

            projected_point = point_on_line(waypoint_a, waypoint_b, robot_xy)

            # Calculate distance from robot to projected point
            distance_to_segment = abs(robot_xy - projected_point)

            if distance_to_segment < min_distance:
                min_distance = distance_to_segment
                closest_segment_idx = i
                closest_projected_point = projected_point

                # Calculate distance along segment from waypoint_a to projected point
                segment_vector = waypoint_b - waypoint_a
                segment_length = abs(segment_vector)
                if segment_length > 0:
                    projection_vector = projected_point - waypoint_a
                    closest_distance_along = abs(projection_vector)
                else:
                    closest_distance_along = 0.0

        return closest_segment_idx, closest_projected_point, closest_distance_along
