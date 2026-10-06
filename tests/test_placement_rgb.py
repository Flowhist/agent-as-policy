"""RGB geometry and rejection checks; no camera or robot connection."""
import sys
from pathlib import Path
from types import SimpleNamespace

import cv2
import numpy as np
import pyrealsense2 as rs
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'scripts'))
from placement_rgb import plane_points, rim_candidates


@pytest.fixture
def calibration():
    intr = rs.intrinsics()
    intr.width, intr.height = 640, 480
    intr.fx = intr.fy = 500
    intr.ppx, intr.ppy = 320, 240
    intr.model = rs.distortion.none
    intr.coeffs = [0]*5
    bc = np.diag([1., -1., -1., 1.])
    bc[2, 3] = 1.
    return SimpleNamespace(intr=intr, bc=bc, normal=np.array([0., 0., 1.]), d=0.)


def test_known_rim_height_changes_ray_intersection_without_changing_floor(calibration):
    points = plane_points([[320, 240], [420, 240]], calibration, .05)
    np.testing.assert_allclose(points, [[0, 0, .05], [.19, 0, .05]], atol=1e-6)
    np.testing.assert_allclose(plane_points([[420, 240]], calibration, 0), [[.2, 0, 0]], atol=1e-6)


def rectangle(shift=0, clipped=False):
    image = np.full((480, 640, 3), 230, np.uint8)
    end = 520 if clipped else 370
    cv2.rectangle(image, (200+shift, 170), (410+shift, end), (30, 30, 30), 3)
    return image


def test_four_supported_static_edges_produce_geometric_candidates(calibration):
    image = rectangle()
    candidates, _ = rim_candidates(image, image, calibration, .05, [0, 0, 640, 480])
    assert candidates
    assert all(min(min(s) for s in row['edge_support_current_and_reference']) >= .60
               for row in candidates)
    assert candidates[0]['center_at_rim_base_m'][2] == pytest.approx(.05)


def test_moving_rectangle_is_not_a_static_placement_region(calibration):
    candidates, _ = rim_candidates(rectangle(45), rectangle(), calibration, .05, [0, 0, 640, 480])
    assert candidates == []


def test_clipped_opening_cannot_invent_a_fourth_rim_edge(calibration):
    image = rectangle(clipped=True)
    candidates, _ = rim_candidates(image, image, calibration, .05, [0, 0, 640, 480])
    assert candidates == []
