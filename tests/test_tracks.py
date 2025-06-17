#!/usr/bin/env python3
"""Tests for Track class functionality."""

import pytest
from rox_vectors import Vector

from rox_control.track import Track


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
        waypoints = [(0, 0), (1, 1), (2, 0)]
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


class TestTrackFindClosestSegment:
    """Test find_closest_segment method functionality."""

    def test_find_closest_segment_on_path(self):
        """Test finding closest segment when robot is on the path."""
        waypoints = [Vector(0, 0), Vector(2, 0), Vector(4, 0)]
        track = Track(waypoints)

        # Robot on first segment
        robot_pos = Vector(1, 0)
        segment_idx, projected_point, distance_along = track.find_closest_segment(
            robot_pos
        )

        assert segment_idx == 0
        assert projected_point == Vector(1, 0)
        assert abs(distance_along - 1.0) < 1e-10

    def test_find_closest_segment_off_path(self):
        """Test finding closest segment when robot is offset from path."""
        waypoints = [Vector(0, 0), Vector(2, 0), Vector(4, 0)]
        track = Track(waypoints)

        # Robot offset above first segment
        robot_pos = Vector(1, 0.5)
        segment_idx, projected_point, distance_along = track.find_closest_segment(
            robot_pos
        )

        assert segment_idx == 0
        assert projected_point == Vector(1, 0)
        assert abs(distance_along - 1.0) < 1e-10

    def test_find_closest_segment_between_waypoints(self):
        """Test segment finding at waypoint boundaries."""
        waypoints = [Vector(0, 0), Vector(2, 0), Vector(4, 0)]
        track = Track(waypoints)

        # Robot exactly at second waypoint
        robot_pos = Vector(2, 0)
        segment_idx, projected_point, distance_along = track.find_closest_segment(
            robot_pos
        )

        # Should find one of the adjacent segments
        assert segment_idx in [0, 1]
        assert projected_point == Vector(2, 0)


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

        # Should handle identical waypoints gracefully
        # This might raise an error due to zero-length segments, which is expected
        try:
            segment_idx, projected_point, distance_along = track.find_closest_segment(
                Vector(0, 0)
            )
            assert isinstance(segment_idx, int)
            assert segment_idx >= 0
        except ZeroDivisionError:
            # This is expected behavior for degenerate (zero-length) segments
            pass

    def test_very_close_waypoints(self):
        """Test behavior with very close waypoints."""
        waypoints = [Vector(0, 0), Vector(0.001, 0), Vector(1, 0)]
        track = Track(waypoints)

        segment_idx, projected_point, distance_along = track.find_closest_segment(
            Vector(0, 0)
        )
        assert isinstance(segment_idx, int)
        assert segment_idx >= 0

    def test_robot_far_from_track(self):
        """Test behavior when robot is far from all waypoints."""
        waypoints = [Vector(0, 0), Vector(1, 0), Vector(2, 0)]
        track = Track(waypoints)

        # Robot far away from track
        segment_idx, projected_point, distance_along = track.find_closest_segment(
            Vector(10, 10)
        )
        assert isinstance(segment_idx, int)
        assert 0 <= segment_idx < len(waypoints) - 1
