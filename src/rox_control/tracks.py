#!/usr/bin/env python3
"""Track class for waypoint management in robotics path following."""

from collections import UserList
from typing import Union

import numpy as np
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
        self._next_idx: int | None = None

    def find_next_idx(self, xy: Vector) -> int:
        """Find next waypoint index for path tracking.

        Core tracking logic that maintains internal state for waypoint progression.
        On first call, finds closest waypoint and advances to next.
        On subsequent calls, progresses when robot moves closer to next waypoint.

        Args:
            xy: Current robot position as Vector

        Returns:
            Index of next waypoint to target
        """
        if self._next_idx is None:  # First time search
            # Find closest waypoint by distance
            distances = [abs(pt - xy) for pt in self.data]
            closest_idx = int(np.argmin(distances))
            self._next_idx = closest_idx + 1
        else:
            # Check if robot has moved closer to next waypoint
            current_idx = self._next_idx
            if current_idx < len(self.data):
                dist_to_current = abs(xy - self.data[current_idx - 1])
                dist_to_next = abs(xy - self.data[current_idx])

                if dist_to_current > dist_to_next:  # Moved closer to next waypoint
                    self._next_idx += 1

        return self._next_idx

    @property
    def target_reached(self) -> bool:
        """Check if end of track has been reached.

        Returns:
            True if robot has progressed beyond final waypoint
        """
        return self._next_idx is not None and self._next_idx >= len(self.data)
