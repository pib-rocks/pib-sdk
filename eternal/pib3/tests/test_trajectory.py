"""Unit tests for trajectory generation and IK utilities.

Tests the pure-math functions in pib3.trajectory that don't require
roboticstoolbox or Webots. This covers interpolation, coordinate mapping,
trajectory serialization, and stroke point generation.
"""

import json
import math
import tempfile
from pathlib import Path

import numpy as np
import pytest

from pib3.config import PaperConfig
from pib3.trajectory import (
    Trajectory,
    _interpolate_failed_points,
    _interpolate_stroke_points,
    _map_to_3d,
    URDF_TO_MOTOR_NAME,
)
from pib3.types import Stroke


# ==================== _interpolate_failed_points ====================


class TestInterpolateFailedPoints:
    """Tests for IK failure interpolation."""

    def test_all_success(self):
        """No interpolation needed when all points succeed."""
        q_traj = [np.array([1.0, 2.0]), np.array([3.0, 4.0]), np.array([5.0, 6.0])]
        flags = [True, True, True]
        result = _interpolate_failed_points(q_traj, flags)
        np.testing.assert_array_equal(result, np.array(q_traj))

    def test_single_failure_between_successes(self):
        """Failed point between two successes is linearly interpolated."""
        q_traj = [np.array([0.0, 0.0]), np.array([99.0, 99.0]), np.array([2.0, 4.0])]
        flags = [True, False, True]
        result = _interpolate_failed_points(q_traj, flags)
        # Interpolation: t = 1/2, so midpoint of [0,0] and [2,4] = [1,2]
        np.testing.assert_allclose(result[1], [1.0, 2.0])

    def test_consecutive_failures(self):
        """Multiple consecutive failures are evenly interpolated."""
        q_traj = [
            np.array([0.0]),
            np.array([99.0]),  # fail
            np.array([99.0]),  # fail
            np.array([99.0]),  # fail
            np.array([4.0]),
        ]
        flags = [True, False, False, False, True]
        result = _interpolate_failed_points(q_traj, flags)
        np.testing.assert_allclose(result[1], [1.0])
        np.testing.assert_allclose(result[2], [2.0])
        np.testing.assert_allclose(result[3], [3.0])

    def test_failure_at_start(self):
        """Failures at the start with no preceding success use next success."""
        q_traj = [np.array([99.0]), np.array([99.0]), np.array([5.0])]
        flags = [False, False, True]
        result = _interpolate_failed_points(q_traj, flags)
        np.testing.assert_allclose(result[0], [5.0])
        np.testing.assert_allclose(result[1], [5.0])

    def test_failure_at_end(self):
        """Failures at the end with no following success use previous success."""
        q_traj = [np.array([3.0]), np.array([99.0]), np.array([99.0])]
        flags = [True, False, False]
        result = _interpolate_failed_points(q_traj, flags)
        np.testing.assert_allclose(result[1], [3.0])
        np.testing.assert_allclose(result[2], [3.0])

    def test_all_failures(self):
        """All failures with no successes — values stay unchanged."""
        q_traj = [np.array([1.0]), np.array([2.0])]
        flags = [False, False]
        result = _interpolate_failed_points(q_traj, flags)
        # No before/after to interpolate from, so values stay as-is
        np.testing.assert_array_equal(result, np.array(q_traj))

    def test_empty(self):
        """Empty trajectory returns empty array."""
        result = _interpolate_failed_points([], [])
        assert len(result) == 0


# ==================== _map_to_3d ====================


class TestMapTo3D:
    """Tests for normalized (u,v) to 3D world coordinate mapping."""

    def test_origin(self):
        """(0, 0) maps to a valid 3D point."""
        paper = PaperConfig(start_x=0.25, size=0.10, height_z=0.85, center_y=-0.34)
        x, y, z = _map_to_3d(0.0, 0.0, paper)
        assert z == pytest.approx(0.85)

    def test_center(self):
        """(0.5, 0.5) maps to the center of the paper."""
        paper = PaperConfig(start_x=0.25, size=0.10, height_z=0.85, center_y=-0.34,
                            drawing_scale=1.0)
        x, y, z = _map_to_3d(0.5, 0.5, paper)
        # center_x = start_x + size/2 = 0.25 + 0.05 = 0.30
        assert x == pytest.approx(0.30)
        assert z == pytest.approx(0.85)

    def test_z_is_paper_height(self):
        """Z coordinate always equals paper height."""
        paper = PaperConfig(start_x=0.0, size=1.0, height_z=1.23, center_y=0.0)
        for u, v in [(0, 0), (0.5, 0.5), (1, 1), (0.3, 0.7)]:
            _, _, z = _map_to_3d(u, v, paper)
            assert z == pytest.approx(1.23)

    def test_drawing_scale_shrinks_range(self):
        """drawing_scale < 1.0 concentrates points toward center."""
        paper_full = PaperConfig(start_x=0.25, size=0.10, height_z=0.85,
                                  center_y=-0.34, drawing_scale=1.0)
        paper_half = PaperConfig(start_x=0.25, size=0.10, height_z=0.85,
                                  center_y=-0.34, drawing_scale=0.5)
        # Corner (1,1) with full scale vs half scale
        x_full, y_full, _ = _map_to_3d(1.0, 1.0, paper_full)
        x_half, y_half, _ = _map_to_3d(1.0, 1.0, paper_half)
        # Center point
        xc, yc, _ = _map_to_3d(0.5, 0.5, paper_full)
        # Half-scale corner should be closer to center than full-scale corner
        dist_full = math.sqrt((x_full - xc)**2 + (y_full - yc)**2)
        dist_half = math.sqrt((x_half - xc)**2 + (y_half - yc)**2)
        assert dist_half < dist_full


# ==================== _interpolate_stroke_points ====================


class TestInterpolateStrokePoints:
    """Tests for stroke point interpolation (densification)."""

    def test_single_point(self):
        """Single point returns itself."""
        stroke = Stroke(points=np.array([[0.5, 0.5]]))
        result = _interpolate_stroke_points(stroke, density=0.01)
        assert len(result) == 1
        assert result[0] == pytest.approx((0.5, 0.5))

    def test_empty_stroke(self):
        """Empty stroke returns empty list."""
        stroke = Stroke(points=np.array([]).reshape(0, 2))
        result = _interpolate_stroke_points(stroke, density=0.01)
        assert len(result) == 0

    def test_two_points_produces_dense_output(self):
        """Two points far apart produce many interpolated points."""
        stroke = Stroke(points=np.array([[0.0, 0.0], [1.0, 0.0]]))
        result = _interpolate_stroke_points(stroke, density=0.1)
        assert len(result) >= 10
        # Check monotonically increasing X
        xs = [p[0] for p in result]
        assert all(xs[i] <= xs[i + 1] for i in range(len(xs) - 1))

    def test_endpoints_preserved(self):
        """First and last interpolated points match stroke endpoints."""
        stroke = Stroke(points=np.array([[0.1, 0.2], [0.8, 0.9]]))
        result = _interpolate_stroke_points(stroke, density=0.01)
        assert result[0] == pytest.approx((0.1, 0.2))
        assert result[-1] == pytest.approx((0.8, 0.9))

    def test_high_density_many_points(self):
        """Very fine density produces many points."""
        stroke = Stroke(points=np.array([[0.0, 0.0], [0.5, 0.5]]))
        fine = _interpolate_stroke_points(stroke, density=0.001)
        coarse = _interpolate_stroke_points(stroke, density=0.1)
        assert len(fine) > len(coarse)


# ==================== Trajectory ====================


class TestTrajectory:
    """Tests for Trajectory data class."""

    def _make_trajectory(self, n_waypoints=5, n_joints=17):
        """Helper to create a simple trajectory."""
        joint_names = [f"joint_{i}" for i in range(n_joints)]
        waypoints = np.random.default_rng(42).uniform(-1, 1, (n_waypoints, n_joints))
        return Trajectory(joint_names=joint_names, waypoints=waypoints)

    def test_len(self):
        """len() returns number of waypoints."""
        traj = self._make_trajectory(n_waypoints=10)
        assert len(traj) == 10

    def test_waypoints_dtype(self):
        """Waypoints are always float64."""
        traj = Trajectory(joint_names=["j"], waypoints=[[1, 2, 3]])
        assert traj.waypoints.dtype == np.float64

    def test_to_webots_format_identity(self):
        """to_webots_format returns waypoints as-is."""
        traj = self._make_trajectory()
        np.testing.assert_array_equal(traj.to_webots_format(), traj.waypoints)

    def test_to_robot_format_centidegrees(self):
        """to_robot_format converts radians to centidegrees."""
        traj = Trajectory(
            joint_names=["j1"],
            waypoints=np.array([[math.pi / 2]]),
        )
        robot_fmt = traj.to_robot_format()
        # π/2 rad = 90° = 9000 centidegrees
        assert robot_fmt[0, 0] == 9000

    def test_json_roundtrip(self):
        """Save and load trajectory preserves data."""
        traj = self._make_trajectory(n_waypoints=3, n_joints=4)
        with tempfile.NamedTemporaryFile(suffix=".json", delete=False) as f:
            path = f.name
        try:
            traj.to_json(path)
            loaded = Trajectory.from_json(path)
            assert loaded.joint_names == traj.joint_names
            np.testing.assert_allclose(loaded.waypoints, traj.waypoints)
            assert "created_at" in loaded.metadata
        finally:
            Path(path).unlink()

    def test_json_contains_metadata(self):
        """Saved JSON includes unit and coordinate frame."""
        traj = self._make_trajectory(n_waypoints=2, n_joints=2)
        with tempfile.NamedTemporaryFile(suffix=".json", delete=False) as f:
            path = f.name
        try:
            traj.to_json(path)
            with open(path) as f:
                data = json.load(f)
            assert data["unit"] == "radians"
            assert data["coordinate_frame"] == "webots"
        finally:
            Path(path).unlink()


# ==================== URDF_TO_MOTOR_NAME ====================


class TestURDFMapping:
    """Tests for the URDF joint index to motor name mapping."""

    def test_all_36_joints_mapped(self):
        """All 36 URDF joints have a motor name."""
        assert len(URDF_TO_MOTOR_NAME) == 36
        for i in range(36):
            assert i in URDF_TO_MOTOR_NAME

    def test_head_joints(self):
        """Head joints are at indices 0-1."""
        assert URDF_TO_MOTOR_NAME[0] == "turn_head_motor"
        assert URDF_TO_MOTOR_NAME[1] == "tilt_forward_motor"

    def test_left_arm_joints(self):
        """Left arm joints are at indices 2-7."""
        assert URDF_TO_MOTOR_NAME[2] == "shoulder_vertical_left"
        assert URDF_TO_MOTOR_NAME[7] == "wrist_left"

    def test_right_arm_joints(self):
        """Right arm joints are at indices 19-24."""
        assert URDF_TO_MOTOR_NAME[19] == "shoulder_vertical_right"
        assert URDF_TO_MOTOR_NAME[24] == "wrist_right"

    def test_finger_proximal_distal_pairs(self):
        """Each finger has two consecutive indices with the same motor name."""
        # Index finger left: joints 11 and 12
        assert URDF_TO_MOTOR_NAME[11] == URDF_TO_MOTOR_NAME[12] == "index_left_stretch"
        # Index finger right: joints 28 and 29
        assert URDF_TO_MOTOR_NAME[28] == URDF_TO_MOTOR_NAME[29] == "index_right_stretch"
