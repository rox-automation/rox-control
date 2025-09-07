#!/usr/bin/env python3
"""Tests for path tracking controllers."""

import math

import pytest
from rox_vectors import Vector

from rox_control.controllers import ControlOutput, PurePursuitA
from rox_control.track import Track
from rox_control.tools.bicicle_model import RobotState


class TestControlOutput:
    """Test ControlOutput dataclass."""

    def test_control_output_creation(self):
        """Test ControlOutput can be created with all fields."""
        output = ControlOutput(
            curvature=0.5,
            velocity=1.0,
            target_point=Vector(1, 2),
            future_position=Vector(0.5, 1.0),
            angle_error=0.1,
            track_complete=False,
        )

        assert output.curvature == 0.5
        assert output.velocity == 1.0
        assert output.target_point == Vector(1, 2)
        assert output.future_position == Vector(0.5, 1.0)
        assert output.angle_error == 0.1
        assert output.track_complete is False


class TestPurePursuitController:
    """Test PurePursuitController class."""

    def test_controller_initialization(self):
        """Test controller initializes with default parameters."""
        controller = PurePursuitA()

        assert controller.look_ahead_distance == 0.2
        assert controller.velocity_vector_length == 0.1
        assert controller.proportional_gain == 1.0
        assert controller.target_speed == 0.1
        assert controller._track is None

    def test_controller_initialization_custom_params(self):
        """Test controller initializes with custom parameters."""
        controller = PurePursuitA(
            look_ahead_distance=0.5,
            velocity_vector_length=0.2,
            proportional_gain=2.0,
            target_speed=0.5,
        )

        assert controller.look_ahead_distance == 0.5
        assert controller.velocity_vector_length == 0.2
        assert controller.proportional_gain == 2.0
        assert controller.target_speed == 0.5

    def test_set_track(self):
        """Test track assignment."""
        controller = PurePursuitA()
        track = Track([Vector(0, 0), Vector(1, 0), Vector(2, 0)])

        controller.set_track(track)

        assert controller._track is track

    def test_control_without_track_raises_error(self):
        """Test control raises error when no track is set."""
        controller = PurePursuitA()
        robot_state = RobotState(x=0.0, y=0.0, theta=0.0)

        with pytest.raises(ValueError, match="No track set"):
            controller.control(robot_state)

    def test_control_with_simple_straight_track(self):
        """Test control with simple straight line track."""
        controller = PurePursuitA(
            look_ahead_distance=0.2,
            velocity_vector_length=0.1,
            proportional_gain=1.0,
            target_speed=0.1,
        )

        # Simple straight track
        track = Track([Vector(0, 0), Vector(2, 0)])
        controller.set_track(track)

        # Robot at origin facing forward
        robot_state = RobotState(x=0.0, y=0.0, theta=0.0)

        output = controller.control(robot_state)

        assert isinstance(output, ControlOutput)
        assert output.velocity == 0.1
        assert output.track_complete is False
        assert isinstance(output.target_point, Vector)
        assert isinstance(output.future_position, Vector)

    def test_control_with_robot_beyond_track(self):
        """Test control when robot is beyond track end."""
        controller = PurePursuitA()
        track = Track([Vector(0, 0), Vector(1, 0)])
        controller.set_track(track)

        # Robot well beyond track end should complete track
        robot_state = RobotState(x=5.0, y=0.0, theta=0.0)
        output = controller.control(robot_state)

        # Should handle gracefully - either complete or continue tracking
        assert isinstance(output.track_complete, bool)
        assert isinstance(output.curvature, float)
        assert isinstance(output.velocity, float)

    def test_proportional_control_zero_error(self):
        """Test proportional controller with zero error."""
        controller = PurePursuitA(proportional_gain=2.0)

        result = controller._proportional_control(0.0, 0.0)

        assert result == 0.0

    def test_proportional_control_positive_error(self):
        """Test proportional controller with positive error."""
        controller = PurePursuitA(proportional_gain=2.0)

        result = controller._proportional_control(0.0, 0.5)

        assert result == -1.0  # 2.0 * (0.0 - 0.5)

    def test_proportional_control_negative_error(self):
        """Test proportional controller with negative error."""
        controller = PurePursuitA(proportional_gain=1.5)

        result = controller._proportional_control(0.0, -0.2)

        assert abs(result - 0.3) < 1e-10  # 1.5 * (0.0 - (-0.2))

    def test_control_integration_with_track_api(self):
        """Test controller integration with new Track API."""
        controller = PurePursuitA(
            velocity_vector_length=0.1,
            look_ahead_distance=0.2,
        )

        # Create a simple L-shaped track
        track = Track([Vector(0, 0), Vector(2, 0), Vector(2, 2)])
        controller.set_track(track)

        # Test robot at start of track
        robot_state = RobotState(x=0.0, y=0.0, theta=0.0)
        output = controller.control(robot_state)

        assert isinstance(output, ControlOutput)
        assert output.track_complete is False
        assert isinstance(output.target_point, Vector)
        assert isinstance(output.future_position, Vector)
        assert isinstance(output.angle_error, float)

    def test_control_with_curved_path(self):
        """Test controller with curved path requiring steering."""
        controller = PurePursuitA(
            velocity_vector_length=0.1,
            look_ahead_distance=0.2,
        )

        # Create curved track
        track = Track([Vector(0, 0), Vector(1, 0), Vector(1, 1)])
        controller.set_track(track)

        # Robot at corner requiring steering
        robot_state = RobotState(x=0.9, y=0.0, theta=0.0)
        output = controller.control(robot_state)

        assert isinstance(output.curvature, float)
        assert output.track_complete is False


class TestIntegrationWithTrack:
    """Integration tests with Track class."""

    def test_full_controller_track_integration(self):
        """Test full integration between controller and track."""
        controller = PurePursuitA(target_speed=0.5)

        # Create L-shaped track
        track = Track(
            [
                Vector(0, 0),
                Vector(2, 0),
                Vector(2, 2),
            ]
        )
        controller.set_track(track)

        # Test at start of track
        robot_state = RobotState(x=0.1, y=0.0, theta=0.0)
        output = controller.control(robot_state)

        assert output.velocity == 0.5
        assert output.track_complete is False
        assert isinstance(output.target_point, Vector)

        # Simulate robot moving along track
        robot_state = RobotState(x=1.8, y=0.0, theta=0.0)
        output = controller.control(robot_state)

        assert output.track_complete is False
        # Should be heading toward the turn
        assert output.target_point.x >= 1.8

    def test_controller_with_minimum_track(self):
        """Test controller with minimum viable track (2 waypoints)."""
        controller = PurePursuitA()
        track = Track([Vector(0, 0), Vector(1, 1)])
        controller.set_track(track)

        robot_state = RobotState(x=0.0, y=0.0, theta=math.pi / 4)
        output = controller.control(robot_state)

        assert output.track_complete is False
        assert output.velocity == controller.target_speed
        assert isinstance(output.angle_error, float)

    def test_controller_progresses_through_track(self):
        """Test that controller works consistently as robot progresses."""
        controller = PurePursuitA()
        track = Track(
            [
                Vector(0, 0),
                Vector(1, 0),
                Vector(2, 0),
                Vector(3, 0),
            ]
        )
        controller.set_track(track)

        # Start near first waypoint
        robot_state = RobotState(x=0.1, y=0.0, theta=0.0)
        output1 = controller.control(robot_state)

        # Move closer to second waypoint
        robot_state = RobotState(x=0.8, y=0.0, theta=0.0)
        output2 = controller.control(robot_state)

        # Both calls should work and track should not be complete
        assert isinstance(output1, ControlOutput)
        assert isinstance(output2, ControlOutput)
        assert not output2.track_complete
