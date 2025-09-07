#!/usr/bin/env python3
"""Pure pursuit A path tracking controller implementation."""

from dataclasses import dataclass
from typing import TYPE_CHECKING

from rox_vectors import Vector

from rox_control.track import Track

if TYPE_CHECKING:  # pragma: no cover
    from rox_control.tools.bicicle_model import RobotState


@dataclass
class ControlOutput:
    """Structured control output from path tracking controllers."""

    curvature: float
    velocity: float
    target_point: Vector
    future_position: Vector
    angle_error: float
    track_complete: bool


class Controller:
    """Pure pursuit A path tracking controller with velocity-projected lookahead."""

    def __init__(
        self,
        look_ahead_distance: float = 0.2,
        velocity_vector_length: float = 0.1,
        proportional_gain: float = 1.0,
        target_speed: float = 0.1,
    ) -> None:
        """Initialize pure pursuit controller with configurable parameters."""
        self.look_ahead_distance = look_ahead_distance
        self.velocity_vector_length = velocity_vector_length
        self.proportional_gain = proportional_gain
        self.target_speed = target_speed
        self._track: Track | None = None

    def set_track(self, track: Track) -> None:
        """Assign track for controller to follow."""
        self._track = track

    def control(self, robot_state: "RobotState") -> ControlOutput:
        """Calculate control output for current robot state.

        Returns: ControlOutput with curvature and velocity commands.
        """
        if self._track is None:
            raise ValueError("No track set. Call set_track() first.")

        # Convert robot position to Vector
        robot_xy = Vector(robot_state.x, robot_state.y)
        robot_heading = robot_state.theta

        # Find closest segment and project robot onto track
        segment_idx, projected_point, distance_along = self._track.find_closest_segment(
            robot_xy
        )

        # Get target point using lookahead distance
        target_point, track_complete = self._get_lookahead_point(
            segment_idx, distance_along, self.look_ahead_distance
        )

        # Check if track is complete
        if track_complete:
            return ControlOutput(
                curvature=0.0,
                velocity=0.0,
                target_point=robot_xy,
                future_position=robot_xy,
                angle_error=0.0,
                track_complete=True,
            )

        # Calculate future position and angle error
        velocity_vector = Vector.from_polar(self.velocity_vector_length, robot_heading)
        future_position = robot_xy + velocity_vector
        angle_error = velocity_vector.angle(target_point - robot_xy)

        # Calculate curvature command using proportional control
        curvature = self._proportional_control(0.0, angle_error)

        return ControlOutput(
            curvature=curvature,
            velocity=self.target_speed,
            target_point=target_point,
            future_position=future_position,
            angle_error=angle_error,
            track_complete=False,
        )

    def _proportional_control(self, target: float, current: float) -> float:
        """Proportional controller for angle error correction.

        Returns: Curvature command.
        """
        return self.proportional_gain * (target - current)

    def _get_lookahead_point(
        self, segment_idx: int, distance_along_segment: float, lookahead_distance: float
    ) -> tuple[Vector, bool]:
        """Get target point at lookahead distance ahead on track.

        Returns: Tuple of (target_point, track_complete).
        """
        if self._track is None:
            raise ValueError("No track set")

        if segment_idx >= len(self._track) - 1:
            # Already at or past the last segment
            return self._track[-1], True

        remaining_lookahead = lookahead_distance
        current_segment_idx = segment_idx
        current_distance_along = distance_along_segment

        # Start from current position on current segment
        while current_segment_idx < len(self._track) - 1:
            waypoint_a = self._track[current_segment_idx]
            waypoint_b = self._track[current_segment_idx + 1]
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
        return self._track[-1], True
