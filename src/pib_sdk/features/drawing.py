"""Image-to-trajectory drawing: turn a sketch into an arm motion over a surface.

Pipeline
--------
:func:`image_to_sketch` traces a raster image into a :class:`Sketch` of
:class:`Stroke`\\ s (normalized ``[0, 1]`` point paths). :func:`sketch_to_trajectory`
then maps each stroke onto a :class:`DrawingSurface` placed in the robot's base
frame and solves inverse kinematics (via :mod:`pib_sdk.kinematics`) point by
point into a :class:`Trajectory`, which :meth:`Trajectory.play` sends to the
robot through a :class:`pib_sdk.control.Write` connection.

Example
-------
    from pib_sdk.control import Write
    from pib_sdk.kinematics import pose_from_xyz_rpy
    from pib_sdk.features.drawing import DrawingSurface, image_to_sketch, sketch_to_trajectory

    sketch = image_to_sketch("logo.png")
    surface = DrawingSurface(
        pose=pose_from_xyz_rpy([-350, 100, 850], rpy_deg=[0, 0, 0]),
        width_mm=200,
        height_mm=200,
    )
    trajectory = sketch_to_trajectory(sketch, surface, side="right")

    writer = Write(host="localhost")
    trajectory.play(writer)

Building a :class:`Sketch` doesn't require the robot or Pillow -- only
:func:`image_to_sketch` needs Pillow (``pip install pib-sdk[drawing]``); you
can also construct :class:`Stroke`/:class:`Sketch` directly from your own
point lists (e.g. from an SVG path or a vision pipeline).
"""

from __future__ import annotations

import logging
import time
from collections.abc import Callable, Sequence
from dataclasses import dataclass
from pathlib import Path

import numpy as np
import pinocchio as pin

from pib_sdk.control import Write, left_arm, right_arm
from pib_sdk.kinematics import ArmKinematics
from pib_sdk.robot_model import ArmSide, coerce_arm_side

logger = logging.getLogger("pib.features.drawing")

# Ink regions larger than this are subsampled before path-ordering, since the
# nearest-neighbour walk below is O(n^2) and strokes are resampled down to a
# handful of points anyway once they reach sketch_to_trajectory().
_MAX_REGION_POINTS = 400


# --------------------------------------------------------------------------- #
# Sketch geometry
# --------------------------------------------------------------------------- #
@dataclass(frozen=True)
class Stroke:
    """A continuous pen path: normalized ``(x, y)`` points, each in ``[0, 1]``."""

    points: tuple[tuple[float, float], ...]

    def __post_init__(self) -> None:
        if len(self.points) < 2:
            raise ValueError("A stroke needs at least 2 points")

    def resampled(self, count: int) -> list[tuple[float, float]]:
        """Return ``count`` points evenly spaced by arc length along the stroke."""
        if count < 2:
            raise ValueError("count must be at least 2")
        points = np.asarray(self.points, dtype=float)
        segment_lengths = np.linalg.norm(np.diff(points, axis=0), axis=1)
        cumulative = np.concatenate([[0.0], np.cumsum(segment_lengths)])
        total_length = cumulative[-1]
        if total_length == 0.0:
            return [(float(points[0, 0]), float(points[0, 1]))] * count
        targets = np.linspace(0.0, total_length, count)
        x = np.interp(targets, cumulative, points[:, 0])
        y = np.interp(targets, cumulative, points[:, 1])
        return list(zip(x.tolist(), y.tolist(), strict=True))


@dataclass(frozen=True)
class Sketch:
    """A collection of strokes, e.g. loaded from an image via :func:`image_to_sketch`."""

    strokes: tuple[Stroke, ...]

    def __post_init__(self) -> None:
        if not self.strokes:
            raise ValueError("A sketch needs at least one stroke")

    @property
    def point_count(self) -> int:
        return sum(len(stroke.points) for stroke in self.strokes)


# --------------------------------------------------------------------------- #
# Image -> Sketch
# --------------------------------------------------------------------------- #
def image_to_sketch(
    image_path: str | Path,
    *,
    threshold: int = 128,
    max_size: int = 200,
    min_stroke_points: int = 4,
) -> Sketch:
    """Trace the dark pixels of an image into a :class:`Sketch` of strokes.

    Requires Pillow (``pip install pib-sdk[drawing]``). The image is
    downsampled to at most ``max_size`` pixels on its longest side, thresholded
    into a dark/light mask (pixels darker than ``threshold`` count as ink),
    split into 8-connected ink regions, and each region is ordered into a path
    with a greedy nearest-neighbour walk.

    This is a simple tracer meant for clean line art (sketches, logos), not a
    replacement for a real vision pipeline -- for photographs or dense
    artwork, preprocess elsewhere (e.g. edge detection, skeletonization) and
    build a :class:`Sketch` directly from the resulting point lists.
    """
    try:
        from PIL import Image
    except ImportError as error:
        raise ImportError(
            "image_to_sketch() requires Pillow; install it with `pip install pib-sdk[drawing]`."
        ) from error

    with Image.open(image_path) as image:
        grayscale = image.convert("L")
        width, height = grayscale.size
        scale = min(1.0, max_size / max(width, height))
        if scale < 1.0:
            grayscale = grayscale.resize(
                (max(1, round(width * scale)), max(1, round(height * scale)))
            )
        mask = np.asarray(grayscale, dtype=np.uint8) < threshold

    strokes = tuple(
        Stroke(tuple(_pixels_to_unit_points(region, mask.shape)))
        for region in _trace_ink_regions(mask)
        if len(region) >= min_stroke_points
    )
    if not strokes:
        raise ValueError(f"No ink found in {image_path} at threshold={threshold}")
    return Sketch(strokes)


def _pixels_to_unit_points(
    pixels: list[tuple[int, int]], shape: tuple[int, int]
) -> list[tuple[float, float]]:
    height, width = shape
    return [(col / max(1, width - 1), 1.0 - row / max(1, height - 1)) for row, col in pixels]


def _trace_ink_regions(mask: np.ndarray) -> list[list[tuple[int, int]]]:
    """Split ``mask`` into 8-connected ink regions, each ordered into a path."""
    visited = np.zeros_like(mask, dtype=bool)
    regions: list[list[tuple[int, int]]] = []
    for start_row, start_col in zip(*np.nonzero(mask), strict=True):
        start_row, start_col = int(start_row), int(start_col)
        if visited[start_row, start_col]:
            continue
        pixels = _flood_fill(mask, visited, start_row, start_col)
        if len(pixels) > _MAX_REGION_POINTS:
            stride = len(pixels) / _MAX_REGION_POINTS
            pixels = [pixels[int(i * stride)] for i in range(_MAX_REGION_POINTS)]
        regions.append(_order_by_nearest_neighbour(pixels))
    return regions


def _flood_fill(
    mask: np.ndarray, visited: np.ndarray, start_row: int, start_col: int
) -> list[tuple[int, int]]:
    height, width = mask.shape
    stack = [(start_row, start_col)]
    visited[start_row, start_col] = True
    pixels: list[tuple[int, int]] = []
    while stack:
        row, col = stack.pop()
        pixels.append((row, col))
        for d_row in (-1, 0, 1):
            for d_col in (-1, 0, 1):
                n_row, n_col = row + d_row, col + d_col
                if (
                    0 <= n_row < height
                    and 0 <= n_col < width
                    and mask[n_row, n_col]
                    and not visited[n_row, n_col]
                ):
                    visited[n_row, n_col] = True
                    stack.append((n_row, n_col))
    return pixels


def _order_by_nearest_neighbour(pixels: list[tuple[int, int]]) -> list[tuple[int, int]]:
    remaining = pixels[1:]
    path = [pixels[0]]
    points = np.asarray(remaining, dtype=float)
    while remaining:
        current = np.asarray(path[-1], dtype=float)
        nearest = int(np.argmin(np.sum((points - current) ** 2, axis=1)))
        path.append(remaining.pop(nearest))
        points = np.delete(points, nearest, axis=0)
    return path


# --------------------------------------------------------------------------- #
# Sketch -> arm trajectory
# --------------------------------------------------------------------------- #
@dataclass(frozen=True)
class DrawingSurface:
    """A flat surface (e.g. a sheet of paper) placed in the robot's base frame.

    ``pose`` (see :func:`pib_sdk.kinematics.pose_from_xyz_rpy`) places the
    surface's origin and orientation: its local +x/+y span the surface, in the
    directions a normalized sketch's x/y map onto, and its local +z is the
    surface normal, pointing away from the surface -- pen-up moves travel
    along +z by ``lift_mm``.
    """

    pose: pin.SE3
    width_mm: float
    height_mm: float
    lift_mm: float = 20.0

    def point_mm(self, u: float, v: float, *, lift: bool = False) -> np.ndarray:
        """3D position (mm, base frame) for normalized surface point ``(u, v)``."""
        local = np.array([u * self.width_mm, v * self.height_mm, self.lift_mm if lift else 0.0])
        return self.pose.translation + self.pose.rotation @ local


@dataclass(frozen=True)
class Trajectory:
    """An ordered sequence of arm joint-angle waypoints (degrees), ready to play."""

    motor_names: tuple[str, ...]
    side: ArmSide
    waypoints_deg: tuple[tuple[float, ...], ...]

    def __len__(self) -> int:
        return len(self.waypoints_deg)

    def play(
        self,
        writer: Write,
        *,
        rate_hz: float = 8.0,
        stop: Callable[[], bool] | None = None,
    ) -> None:
        """Send each waypoint to ``writer`` in order, at roughly ``rate_hz`` moves/second."""
        arm_token = right_arm if self.side is ArmSide.RIGHT else left_arm
        interval = 1.0 / rate_hz
        for angles_deg in self.waypoints_deg:
            if stop is not None and stop():
                break
            writer.move(arm_token, *angles_deg)
            time.sleep(interval)


def sketch_to_trajectory(
    sketch: Sketch,
    surface: DrawingSurface,
    side: ArmSide | str = ArmSide.RIGHT,
    *,
    rpy_deg: Sequence[float] | None = None,
    points_per_stroke: int = 30,
    initial_guess_deg: Sequence[float] | None = None,
    ik_options: dict | None = None,
) -> Trajectory:
    """Convert ``sketch`` into an arm :class:`Trajectory` over ``surface``.

    Each stroke becomes an approach above its start point, a touch-down, the
    resampled path along the surface, and a lift-off -- so consecutive strokes
    don't drag the pen across the surface in between. Consecutive IK solves
    are warm-started from the previous waypoint, which keeps the motion smooth
    and each solve fast; points where IK fails to converge are skipped with a
    warning rather than aborting the whole trajectory.

    ``ik_options`` are forwarded to :meth:`ArmKinematics.inverse` (e.g.
    ``tolerance``, ``max_iterations``, ``restarts``), which defaults match
    :func:`pib_sdk.kinematics.ik`.
    """
    resolved_side = coerce_arm_side(side)
    kinematics = ArmKinematics(resolved_side)
    options = dict(ik_options or {})

    guess = (
        np.asarray(initial_guess_deg, dtype=float)
        if initial_guess_deg is not None
        else np.zeros(kinematics.degrees_of_freedom)
    )
    waypoints: list[np.ndarray] = []

    def solve(u: float, v: float, *, lift: bool) -> None:
        nonlocal guess
        xyz = surface.point_mm(u, v, lift=lift)
        try:
            angles = kinematics.inverse(
                xyz=xyz, rpy_deg=rpy_deg, initial_guess_deg=guess, **options
            )
        except ValueError:
            logger.warning("Skipping unreachable point (u=%.3f, v=%.3f, lift=%s)", u, v, lift)
            return
        guess = angles
        waypoints.append(angles)

    for stroke in sketch.strokes:
        path = stroke.resampled(points_per_stroke)
        start_u, start_v = path[0]
        end_u, end_v = path[-1]
        solve(start_u, start_v, lift=True)
        for u, v in path:
            solve(u, v, lift=False)
        solve(end_u, end_v, lift=True)

    if not waypoints:
        raise ValueError("Inverse kinematics failed to reach every point of the sketch")

    return Trajectory(
        motor_names=tuple(kinematics.motor_names),
        side=resolved_side,
        waypoints_deg=tuple(tuple(angles) for angles in waypoints),
    )
