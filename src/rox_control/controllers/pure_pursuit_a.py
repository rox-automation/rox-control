#!/usr/bin/env python3
"""Pure pursuit A path tracking controller implementation."""

from dataclasses import dataclass
from typing import TYPE_CHECKING, Optional

from rox_vectors import Vector

from rox_control.tracks import Track

if TYPE_CHECKING:  # pragma: no cover
    from tools.bicicle_model import RobotState


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
    """Pure pursuit A path tracking controller.

    Implements a velocity-projected pure pursuit algorithm for smooth path following
    with configurable lookahead parameters and proportional control.
    """

    def __init__(
        self,
        look_ahead_distance: float = 0.2,
        velocity_vector_length: float = 0.1,
        proportional_gain: float = 1.0,
        target_speed: float = 0.1,
    ) -> None:
        """Initialize pure pursuit controller.

        Args:
            look_ahead_distance: Lookahead distance for target calculation
            velocity_vector_length: Robot velocity projection length
            proportional_gain: Steering control gain
            target_speed: Desired robot velocity
        """
        self.look_ahead_distance = look_ahead_distance
        self.velocity_vector_length = velocity_vector_length
        self.proportional_gain = proportional_gain
        self.target_speed = target_speed
        self._track: Optional[Track] = None

    def set_track(self, track: Track) -> None:
        """Assign track for controller to follow.

        Args:
            track: Track object containing waypoints to follow
        """
        self._track = track

    def control(self, robot_state: "RobotState") -> ControlOutput:
        """Calculate control output for robot state.

        Args:
            robot_state: Current robot state (RobotState object)

        Returns:
            ControlOutput with control commands and debug information

        Raises:
            ValueError: If no track has been set
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
        target_point, track_complete = self._track.get_lookahead_point(
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
        """Proportional controller for angle error.

        Ported from reference implementation's proportional_control function.

        Args:
            target: Target angle (typically 0.0 for pure pursuit)
            current: Current angle error

        Returns:
            Control output (curvature command)
        """
        return self.proportional_gain * (target - current)
