#!/usr/bin/env python3
"""Unit tests for track generator functions."""

import pytest
from rox_vectors import Vector

from rox_control.tools.tracks import generate_track


class TestGenerateTrack:
    """Test the main generate_track function."""

    def test_unsupported_track_type(self):
        """Test error handling for unsupported track types."""
        with pytest.raises(ValueError, match="Unsupported track type: invalid"):
            generate_track("invalid")


class TestCircleTrack:
    """Test circular track generation."""

    def test_default_circle(self):
        """Test circle with default parameters."""
        track = generate_track("circle")
        assert len(track) == 16

        # Check that all points are approximately on unit circle
        waypoints = list(track)
        for wp in waypoints:
            distance_from_origin = abs(wp - Vector(0, 0))
            assert abs(distance_from_origin - 1.0) < 1e-10

    def test_custom_radius(self):
        """Test circle with custom radius."""
        track = generate_track("circle", radius=2.0)
        waypoints = list(track)

        for wp in waypoints:
            distance_from_origin = abs(wp - Vector(0, 0))
            assert abs(distance_from_origin - 2.0) < 1e-10

    def test_custom_center(self):
        """Test circle with custom center."""
        center = Vector(1.0, 2.0)
        track = generate_track("circle", center=center)
        waypoints = list(track)

        for wp in waypoints:
            distance_from_center = abs(wp - center)
            assert abs(distance_from_center - 1.0) < 1e-10

    def test_custom_resolution(self):
        """Test circle with custom resolution."""
        track = generate_track("circle", resolution=8)
        assert len(track) == 8

    def test_invalid_radius(self):
        """Test error handling for invalid radius."""
        with pytest.raises(ValueError, match="Radius must be positive"):
            generate_track("circle", radius=-1.0)

        with pytest.raises(ValueError, match="Radius must be positive"):
            generate_track("circle", radius=0.0)

    def test_invalid_resolution(self):
        """Test error handling for invalid resolution."""
        with pytest.raises(ValueError, match="Resolution must be at least 3"):
            generate_track("circle", resolution=2)


class TestFigure8Track:
    """Test figure-8 track generation."""

    def test_default_figure8(self):
        """Test figure-8 with default parameters."""
        track = generate_track("figure8")
        assert len(track) == 32

        waypoints = list(track)
        # Figure-8 should be symmetric around origin
        assert len(waypoints) > 0

    def test_custom_size(self):
        """Test figure-8 with custom size."""
        track = generate_track("figure8", size=2.0)
        waypoints = list(track)

        # Larger size should produce points further from origin
        max_distance = max(abs(wp - Vector(0, 0)) for wp in waypoints)
        assert max_distance > 1.0  # Should be larger than default

    def test_custom_resolution(self):
        """Test figure-8 with custom resolution."""
        track = generate_track("figure8", resolution=16)
        assert len(track) == 16

    def test_invalid_size(self):
        """Test error handling for invalid size."""
        with pytest.raises(ValueError, match="Size must be positive"):
            generate_track("figure8", size=-1.0)

        with pytest.raises(ValueError, match="Size must be positive"):
            generate_track("figure8", size=0.0)

    def test_invalid_resolution(self):
        """Test error handling for invalid resolution."""
        with pytest.raises(ValueError, match="Resolution must be at least 6"):
            generate_track("figure8", resolution=5)


class TestTrackIntegration:
    """Test integration with Track class."""

    def test_track_properties(self):
        """Test that generated tracks have proper Track properties."""
        track = generate_track("square")

        # Should be able to use Track methods
        assert hasattr(track, "find_closest_segment")

        # Should have at least 2 waypoints (Track requirement)
        assert len(track) >= 2

    def test_all_track_types_return_valid_tracks(self):
        """Test that all track types return valid Track objects."""
        track_types = ["square", "circle", "figure8"]

        for track_type in track_types:
            track = generate_track(track_type)
            assert len(track) >= 2  # Track class requirement

            # Should be able to access waypoints
            waypoints = list(track)
            assert all(isinstance(wp, Vector) for wp in waypoints)
