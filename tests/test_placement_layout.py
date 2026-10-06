"""Offline layout geometry; no hardware access."""
import sys
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pyrealsense2 as rs

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'scripts'))
from placement_layout import project, rectangle
from placement_rgb import plane_points


def test_inverse_brown_projection_round_trip():
    intr = rs.intrinsics()
    intr.width, intr.height = 1280, 720
    intr.fx = intr.fy = 650
    intr.ppx, intr.ppy = 640, 360
    intr.model = rs.distortion.inverse_brown_conrady
    intr.coeffs = [.05, -.02, .001, -.002, .005]
    transform = np.diag([1., -1., -1., 1.])
    transform[2, 3] = .8
    calibration = SimpleNamespace(intr=intr, bc=transform, normal=np.array([0., 0., 1.]), d=0.)
    pixels = np.array([[543., 467.], [752., 479.], [563., 347.], [748., 357.]])
    for height in (0., .05, .12):
        points = plane_points(pixels, calibration, height)
        np.testing.assert_allclose(project(points, calibration), pixels, atol=.001)


def test_rectangle_preserves_size_under_rotated_axes():
    angle = .4
    axes = np.array([[np.cos(angle), np.sin(angle)], [-np.sin(angle), np.cos(angle)]])
    points = rectangle([.1, -.55], axes, [.09, .03])
    np.testing.assert_allclose(points.mean(0), [.1, -.55])
    np.testing.assert_allclose(np.linalg.norm(np.roll(points, -1, axis=0)-points, axis=1), [.09, .03, .09, .03])
