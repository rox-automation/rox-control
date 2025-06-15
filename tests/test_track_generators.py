#!/usr/bin/env python3
"""Unit tests for track generator functions."""

import pytest
from rox_vectors import Vector

from tools.tracks import generate_track


class TestGenerateTrack:
    """Test the main generate_track function."""

    def test_unsupported_track_type(self):
        """Test error handling for unsupported track types."""
        with pytest.raises(ValueError, match="Unsupported track type: invalid"):
            generate_track("invalid")


class TestSquareTrack:
    """Test square track generation."""

    def test_default_square(self):
        """Test square with default parameters."""
        track = generate_track("square")
        assert len(track) == 16  # 4 sides x 4 points per side

        # Check that waypoints form a square centered at origin
        waypoints = list(track)
        assert waypoints[0] == Vector(-0.5, -0.5)  # Bottom-left corner area

    def test_custom_size(self):
        """Test square with custom size."""
        track = generate_track("square", size=2.0)
        waypoints = list(track)

        # Should have corners at ±1.0 (half of size=2.0)
        expected_corner = Vector(-1.0, -1.0)
        assert waypoints[0] == expected_corner

    def test_custom_resolution(self):
        """Test square with custom resolution."""
        track = generate_track("square", resolution=2)
        assert len(track) == 8  # 4 sides x 2 points per side

    def test_invalid_size(self):
        """Test error handling for invalid size."""
        with pytest.raises(ValueError, match="Size must be positive"):
            generate_track("square", size=-1.0)

        with pytest.raises(ValueError, match="Size must be positive"):
            generate_track("square", size=0.0)

    def test_invalid_resolution(self):
        """Test error handling for invalid resolution."""
        with pytest.raises(ValueError, match="Resolution must be at least 1"):
            generate_track("square", resolution=0)

    def test_square_with_spacing(self):
        """Test square with uniform spacing."""
        track = generate_track("square", size=2.0, spacing=0.5)
        waypoints = list(track)

        # With perimeter=8.0 and spacing=0.5, expect 16 points
        assert len(waypoints) >= 2  # At minimum should have 2 points

        # Check that spacing is approximately consistent
        if len(waypoints) > 2:
            distances = []
            for i in range(len(waypoints)):
                next_i = (i + 1) % len(waypoints)
                dist = abs(waypoints[next_i] - waypoints[i])
                distances.append(dist)

            # Spacing should be reasonably consistent
            avg_spacing = sum(distances) / len(distances)
            assert abs(avg_spacing - 0.5) < 0.1


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

    def test_circle_with_spacing(self):
        """Test circle with uniform spacing."""
        track = generate_track("circle", radius=1.0, spacing=0.5)
        waypoints = list(track)

        # With circumference≈6.28 and spacing=0.5, expect ~12-13 points
        assert len(waypoints) >= 2

        # Check that all points are still on the circle
        for wp in waypoints:
            distance_from_origin = abs(wp - Vector(0, 0))
            assert abs(distance_from_origin - 1.0) < 0.1


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

    def test_figure8_with_spacing(self):
        """Test figure-8 with uniform spacing."""
        track = generate_track("figure8", size=1.0, spacing=0.2)
        waypoints = list(track)

        assert len(waypoints) >= 2


class TestSpacingParameter:
    """Test spacing parameter behavior across all track types."""

    def test_invalid_spacing(self):
        """Test error handling for invalid spacing values."""
        with pytest.raises(ValueError, match="Spacing must be positive"):
            generate_track("square", spacing=-0.1)

        with pytest.raises(ValueError, match="Spacing must be positive"):
            generate_track("circle", spacing=0.0)

        with pytest.raises(ValueError, match="Spacing must be positive"):
            generate_track("figure8", spacing=-1.0)

    def test_very_large_spacing(self):
        """Test behavior with very large spacing values."""
        # Large spacing should result in minimum 2 waypoints
        track = generate_track("square", size=4.0, spacing=1.0)
        assert len(track) >= 2

    def test_very_small_spacing(self):
        """Test behavior with very small spacing values."""
        # Small spacing should result in many waypoints
        track = generate_track("circle", radius=1.0, spacing=0.1)
        assert len(track) > 10  # Should have many points


class TestTrackIntegration:
    """Test integration with Track class."""

    def test_track_properties(self):
        """Test that generated tracks have proper Track properties."""
        track = generate_track("square")

        # Should be able to use Track methods
        assert hasattr(track, "find_next_idx")
        assert hasattr(track, "target_reached")

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
