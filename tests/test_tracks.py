#!/usr/bin/env python3
"""Tests for Track class functionality."""

import pytest
from rox_control.tracks import Track
from rox_vectors import Vector


class TestTrackInitialization:
    """Test Track class initialization and validation."""

    def test_init_with_vectors(self):
        """Test initialization with Vector objects."""
        waypoints = [Vector(0, 0), Vector(1, 1), Vector(2, 0)]
        track = Track(waypoints)

        assert len(track) == 3
        assert all(isinstance(wp, Vector) for wp in track)
        assert track[0] == Vector(0, 0)
        assert track[1] == Vector(1, 1)
        assert track[2] == Vector(2, 0)

    def test_init_with_tuples(self):
        """Test automatic conversion from tuples to Vectors."""
        waypoints = [(0, 0), (1, 1), (2, 0)]
        track = Track(waypoints)

        assert len(track) == 3
        assert all(isinstance(wp, Vector) for wp in track)
        assert track[0] == Vector(0, 0)
        assert track[1] == Vector(1, 1)
        assert track[2] == Vector(2, 0)

    def test_init_with_lists(self):
        """Test automatic conversion from lists to Vectors."""
        waypoints = [[0, 0], [1, 1], [2, 0]]
        track = Track(waypoints)

        assert len(track) == 3
        assert all(isinstance(wp, Vector) for wp in track)
        assert track[0] == Vector(0, 0)
        assert track[1] == Vector(1, 1)
        assert track[2] == Vector(2, 0)

    def test_init_mixed_types(self):
        """Test initialization with mixed Vector, tuple, and list types."""
        waypoints = [Vector(0, 0), (1, 1), [2, 0]]
        track = Track(waypoints)

        assert len(track) == 3
        assert all(isinstance(wp, Vector) for wp in track)
        assert track[0] == Vector(0, 0)
        assert track[1] == Vector(1, 1)
        assert track[2] == Vector(2, 0)

    def test_minimum_waypoints_validation(self):
        """Test validation of minimum 2 waypoints requirement."""
        # Empty list should raise ValueError
        with pytest.raises(ValueError, match="at least 2 waypoints"):
            Track([])

        # Single waypoint should raise ValueError
        with pytest.raises(ValueError, match="at least 2 waypoints"):
            Track([Vector(0, 0)])

        # Two waypoints should work
        track = Track([Vector(0, 0), Vector(1, 1)])
        assert len(track) == 2


class TestTrackFindNextIdx:
    """Test find_next_idx method functionality."""

    def test_first_call_finds_closest(self):
        """Test that first call finds closest waypoint and advances to next."""
        waypoints = [Vector(0, 0), Vector(2, 0), Vector(4, 0)]
        track = Track(waypoints)

        # Robot closer to second waypoint
        robot_pos = Vector(1.8, 0)
        next_idx = track.find_next_idx(robot_pos)

        # Should find closest (index 1) and return next (index 2)
        assert next_idx == 2
        assert track._next_idx == 2

    def test_first_call_closest_is_first(self):
        """Test first call when robot is closest to first waypoint."""
        waypoints = [Vector(0, 0), Vector(2, 0), Vector(4, 0)]
        track = Track(waypoints)

        # Robot closest to first waypoint
        robot_pos = Vector(0.1, 0)
        next_idx = track.find_next_idx(robot_pos)

        # Should find closest (index 0) and return next (index 1)
        assert next_idx == 1
        assert track._next_idx == 1

    def test_progression_logic(self):
        """Test waypoint progression when robot moves closer to next waypoint."""
        waypoints = [Vector(0, 0), Vector(2, 0), Vector(4, 0)]
        track = Track(waypoints)

        # Initialize at position closer to first waypoint
        robot_pos = Vector(0.5, 0)
        next_idx = track.find_next_idx(robot_pos)
        assert next_idx == 1  # Should target second waypoint

        # Move robot closer to second waypoint but still closer to first
        robot_pos = Vector(0.8, 0)
        next_idx = track.find_next_idx(robot_pos)
        assert next_idx == 1  # Should still target second waypoint

        # Move robot closer to second waypoint than first
        robot_pos = Vector(1.2, 0)
        next_idx = track.find_next_idx(robot_pos)
        assert next_idx == 2  # Should advance to third waypoint

    def test_no_regression(self):
        """Test that waypoint index doesn't regress."""
        waypoints = [Vector(0, 0), Vector(2, 0), Vector(4, 0)]
        track = Track(waypoints)

        # Start near second waypoint
        robot_pos = Vector(1.8, 0)
        next_idx = track.find_next_idx(robot_pos)
        assert next_idx == 2

        # Move back toward first waypoint
        robot_pos = Vector(0.5, 0)
        next_idx = track.find_next_idx(robot_pos)
        assert next_idx == 2  # Should not regress

    def test_beyond_final_waypoint(self):
        """Test behavior when robot progresses beyond final waypoint."""
        waypoints = [Vector(0, 0), Vector(2, 0)]
        track = Track(waypoints)

        # Start near second waypoint and progress beyond
        robot_pos = Vector(3, 0)
        next_idx = track.find_next_idx(robot_pos)

        # Should return index beyond array length
        assert next_idx == 2
        assert track.target_reached


class TestTrackTargetReached:
    """Test target_reached property functionality."""

    def test_initial_state(self):
        """Test target_reached is False initially."""
        waypoints = [Vector(0, 0), Vector(2, 0)]
        track = Track(waypoints)

        assert not track.target_reached

    def test_not_reached_during_progression(self):
        """Test target_reached is False during normal progression."""
        waypoints = [Vector(0, 0), Vector(2, 0), Vector(4, 0)]
        track = Track(waypoints)

        # Progress through waypoints
        track.find_next_idx(Vector(0.1, 0))  # Should target waypoint 1
        assert not track.target_reached

        track.find_next_idx(Vector(1.5, 0))  # Should target waypoint 2
        assert not track.target_reached

    def test_reached_at_end(self):
        """Test target_reached is True when all waypoints completed."""
        waypoints = [Vector(0, 0), Vector(2, 0)]
        track = Track(waypoints)

        # Progress beyond final waypoint
        track.find_next_idx(Vector(3, 0))
        assert track.target_reached


class TestTrackUserListBehavior:
    """Test Track inherits UserList functionality correctly."""

    def test_indexing(self):
        """Test list-like indexing behavior."""
        waypoints = [Vector(0, 0), Vector(1, 1), Vector(2, 0)]
        track = Track(waypoints)

        assert track[0] == Vector(0, 0)
        assert track[1] == Vector(1, 1)
        assert track[2] == Vector(2, 0)
        assert track[-1] == Vector(2, 0)

    def test_length(self):
        """Test len() functionality."""
        waypoints = [Vector(0, 0), Vector(1, 1), Vector(2, 0)]
        track = Track(waypoints)

        assert len(track) == 3

    def test_iteration(self):
        """Test iteration over waypoints."""
        waypoints = [Vector(0, 0), Vector(1, 1), Vector(2, 0)]
        track = Track(waypoints)

        waypoint_list = list(track)
        assert len(waypoint_list) == 3
        assert waypoint_list == waypoints

    def test_append(self):
        """Test appending new waypoints."""
        waypoints = [Vector(0, 0), Vector(1, 1)]
        track = Track(waypoints)

        track.append(Vector(2, 0))
        assert len(track) == 3
        assert track[2] == Vector(2, 0)


class TestTrackEdgeCases:
    """Test Track behavior in edge cases."""

    def test_identical_waypoints(self):
        """Test behavior with identical waypoints."""
        waypoints = [Vector(0, 0), Vector(0, 0), Vector(1, 0)]
        track = Track(waypoints)

        # Should handle identical waypoints without error
        next_idx = track.find_next_idx(Vector(0, 0))
        assert isinstance(next_idx, int)
        assert next_idx >= 1

    def test_very_close_waypoints(self):
        """Test behavior with very close waypoints."""
        waypoints = [Vector(0, 0), Vector(0.001, 0), Vector(1, 0)]
        track = Track(waypoints)

        next_idx = track.find_next_idx(Vector(0, 0))
        assert isinstance(next_idx, int)
        assert next_idx >= 1

    def test_robot_far_from_track(self):
        """Test behavior when robot is far from all waypoints."""
        waypoints = [Vector(0, 0), Vector(1, 0), Vector(2, 0)]
        track = Track(waypoints)

        # Robot far away from track
        next_idx = track.find_next_idx(Vector(10, 10))
        assert isinstance(next_idx, int)
        assert 1 <= next_idx <= len(waypoints)
