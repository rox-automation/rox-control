#!/usr/bin/env python3
"""Track class for waypoint management in robotics path following."""

from collections import UserList
from typing import Union

from rox_vectors import Vector


class Track(UserList):
    """Navigation waypoints collection with path tracking functionality.

    Inherits from UserList for list-like behavior with indexing support.
    Uses rox_vectors.Vector for waypoints with full type annotation coverage.
    """

    def __init__(
        self, waypoints: list[Union[Vector, tuple[float, float], list[float]]]
    ):
        """Initialize Track with waypoints.

        Args:
            waypoints: List of waypoints as Vector objects, (x, y) tuples, or [x, y] lists

        Raises:
            ValueError: If fewer than 2 waypoints provided
        """
        # Convert all waypoints to Vector objects
        converted_waypoints = [
            Vector(*wp) if isinstance(wp, (tuple, list)) else wp for wp in waypoints
        ]

        # Validate minimum waypoint requirement
        if len(converted_waypoints) < 2:
            raise ValueError("Track must contain at least 2 waypoints")

        super().__init__(converted_waypoints)

    def find_closest_segment(self, robot_xy: Vector) -> tuple[int, Vector, float]:
        """Find closest track segment and project robot onto it.

        Args:
            robot_xy: Current robot position as Vector

        Returns:
            Tuple of (segment_index, projected_point, distance_along_segment_meters)
        """
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

    def get_lookahead_point(
        self, segment_idx: int, distance_along_segment: float, lookahead_distance: float
    ) -> tuple[Vector, bool]:
        """Get target point at lookahead distance ahead on track.

        Args:
            segment_idx: Starting segment index
            distance_along_segment: Distance along segment in meters
            lookahead_distance: Lookahead distance in meters

        Returns:
            Tuple of (target_point, track_complete)
        """
        if segment_idx >= len(self.data) - 1:
            # Already at or past the last segment
            return self.data[-1], True

        remaining_lookahead = lookahead_distance
        current_segment_idx = segment_idx
        current_distance_along = distance_along_segment

        # Start from current position on current segment
        while current_segment_idx < len(self.data) - 1:
            waypoint_a = self.data[current_segment_idx]
            waypoint_b = self.data[current_segment_idx + 1]
            segment_vector = waypoint_b - waypoint_a
            segment_length = abs(segment_vector)

            # Remaining distance in current segment
            remaining_in_segment = segment_length - current_distance_along

            if remaining_lookahead <= remaining_in_segment:
                # Target point is within current segment
                if segment_length > 0:
                    segment_direction = segment_vector / segment_length
                    target_distance_along = current_distance_along + remaining_lookahead
                    target_point = (
                        waypoint_a + segment_direction * target_distance_along
                    )
                else:
                    target_point = waypoint_a
                return target_point, False

            # Move to next segment
            remaining_lookahead -= remaining_in_segment
            current_segment_idx += 1
            current_distance_along = 0.0

        # Lookahead extends beyond track end
        return self.data[-1], True
