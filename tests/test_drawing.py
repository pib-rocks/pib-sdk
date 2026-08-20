"""Tests for pib_sdk.features.drawing."""

from __future__ import annotations

import numpy as np
import pytest

from pib_sdk.features.drawing import (
    DrawingSurface,
    Sketch,
    Stroke,
    Trajectory,
    image_to_sketch,
    sketch_to_trajectory,
)
from pib_sdk.kinematics import ArmKinematics, pose_from_xyz_rpy
from pib_sdk.robot_model import RIGHT_ARM, ArmSide

# --------------------------------------------------------------------------- #
# Stroke / Sketch                                                              #
# --------------------------------------------------------------------------- #


def test_stroke_requires_at_least_two_points():
    with pytest.raises(ValueError):
        Stroke(((0.0, 0.0),))


def test_stroke_resampled_is_evenly_spaced_along_a_straight_line():
    stroke = Stroke(((0.0, 0.0), (1.0, 0.0)))
    points = stroke.resampled(5)
    assert len(points) == 5
    xs = [x for x, _ in points]
    assert xs == sorted(xs)
    assert xs[0] == pytest.approx(0.0)
    assert xs[-1] == pytest.approx(1.0)
    assert np.allclose(np.diff(xs), 0.25)


def test_stroke_resampled_handles_a_degenerate_zero_length_stroke():
    stroke = Stroke(((0.5, 0.5), (0.5, 0.5)))
    points = stroke.resampled(3)
    assert points == [(0.5, 0.5)] * 3


def test_stroke_resampled_rejects_too_few_points():
    stroke = Stroke(((0.0, 0.0), (1.0, 1.0)))
    with pytest.raises(ValueError):
        stroke.resampled(1)


def test_sketch_requires_at_least_one_stroke():
    with pytest.raises(ValueError):
        Sketch(())


def test_sketch_point_count_sums_its_strokes():
    sketch = Sketch(
        (
            Stroke(((0.0, 0.0), (1.0, 0.0), (1.0, 1.0))),
            Stroke(((0.0, 0.0), (0.5, 0.5))),
        )
    )
    assert sketch.point_count == 5


# --------------------------------------------------------------------------- #
# DrawingSurface                                                               #
# --------------------------------------------------------------------------- #


def test_drawing_surface_point_mm_at_identity_pose():
    surface = DrawingSurface(
        pose=pose_from_xyz_rpy([100.0, 0.0, 0.0]),
        width_mm=200.0,
        height_mm=100.0,
        lift_mm=15.0,
    )
    assert np.allclose(surface.point_mm(0.0, 0.0), [100.0, 0.0, 0.0])
    assert np.allclose(surface.point_mm(1.0, 1.0), [300.0, 100.0, 0.0])
    assert np.allclose(surface.point_mm(0.0, 0.0, lift=True), [100.0, 0.0, 15.0])


# --------------------------------------------------------------------------- #
# Sketch -> Trajectory                                                        #
# --------------------------------------------------------------------------- #


def test_sketch_to_trajectory_produces_a_playable_trajectory():
    reachable_center = ArmKinematics(ArmSide.RIGHT).forward(
        RIGHT_ARM.named_configurations_deg["observe"]
    )
    surface = DrawingSurface(
        pose=pose_from_xyz_rpy(reachable_center.translation),
        width_mm=30.0,
        height_mm=30.0,
        lift_mm=10.0,
    )
    sketch = Sketch((Stroke(((0.0, 0.0), (1.0, 1.0))),))

    trajectory = sketch_to_trajectory(sketch, surface, side="right", points_per_stroke=3)

    assert isinstance(trajectory, Trajectory)
    assert trajectory.side is ArmSide.RIGHT
    assert trajectory.motor_names == tuple(ArmKinematics(ArmSide.RIGHT).motor_names)
    assert len(trajectory) > 0


def test_trajectory_play_sends_one_move_per_waypoint():
    trajectory = Trajectory(
        motor_names=tuple(RIGHT_ARM.motor_names),
        side=ArmSide.RIGHT,
        waypoints_deg=((0.0, 0.0, 0.0, 0.0, 0.0, 0.0), (1.0, 2.0, 3.0, 4.0, 5.0, 6.0)),
    )
    calls: list[tuple] = []

    class _FakeWriter:
        def move(self, *args):
            calls.append(args)
            return True

    trajectory.play(_FakeWriter(), rate_hz=1000.0)

    assert len(calls) == 2
    for call, angles in zip(calls, trajectory.waypoints_deg, strict=True):
        token, *sent_angles = call
        assert token.name == "right_arm"
        assert tuple(sent_angles) == angles


# --------------------------------------------------------------------------- #
# Image -> Sketch                                                              #
# --------------------------------------------------------------------------- #


def test_image_to_sketch_traces_a_diagonal_line(tmp_path):
    pil_image_module = pytest.importorskip("PIL.Image")
    image = pil_image_module.new("L", (50, 50), color=255)
    for i in range(50):
        image.putpixel((i, i), 0)
    image_path = tmp_path / "diagonal.png"
    image.save(image_path)

    sketch = image_to_sketch(image_path, min_stroke_points=2)

    assert isinstance(sketch, Sketch)
    assert sketch.point_count >= 2
    all_x = [x for stroke in sketch.strokes for x, _ in stroke.points]
    all_y = [y for stroke in sketch.strokes for _, y in stroke.points]
    assert min(all_x) >= 0.0 and max(all_x) <= 1.0
    assert min(all_y) >= 0.0 and max(all_y) <= 1.0


def test_image_to_sketch_raises_without_pillow_available(monkeypatch, tmp_path):
    import builtins

    real_import = builtins.__import__

    def _blocked_import(name, *args, **kwargs):
        if name == "PIL":
            raise ImportError("simulated missing Pillow")
        return real_import(name, *args, **kwargs)

    monkeypatch.setattr(builtins, "__import__", _blocked_import)

    with pytest.raises(ImportError, match="pib-sdk\\[drawing\\]"):
        image_to_sketch(tmp_path / "does-not-matter.png")
