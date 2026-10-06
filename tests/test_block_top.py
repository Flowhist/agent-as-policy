"""Regression images distinguish a top from the same-color front face."""
from pathlib import Path
from types import SimpleNamespace
import sys

import cv2
import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from block_top import TopFaceNotFound, detect_top, TopPlane


class Plane:
    intr = SimpleNamespace(width=320, height=240)
    bc = np.eye(4)
    bc[:3, 3] = [.145, 1., 1.]
    normal = np.array([0., 0., 1.])

    def points(self, pixels):
        return np.column_stack([np.asarray(pixels)/1000, np.full(len(pixels), .02961)])


def scene(side_height=15, separator=True):
    image = np.full((240, 320, 3), 230, np.uint8)
    cv2.rectangle(image, (100, 80), (190, 110+side_height), (0, 0, 90 if separator else 190), -1)
    cv2.rectangle(image, (100, 80), (190, 110), (0, 0, 190), -1)
    return image


@pytest.mark.parametrize('side_height', [8, 15, 25])
def test_visible_side_is_excluded_from_top_center(side_height):
    result = detect_top(scene(side_height), Plane())
    np.testing.assert_allclose(np.array(result['center_base_m'])[:2]*1000, [145, 95], atol=1)
    assert max(p[1] for p in result['corners_px']) < 112


@pytest.mark.parametrize('side_height', [2, 15])
def test_no_top_side_boundary_rejected_instead_of_silhouette_fallback(side_height):
    with pytest.raises(TopFaceNotFound):
        detect_top(scene(side_height=side_height, separator=False), Plane())


def test_gripper_occlusion_rejects_center():
    image = scene()
    cv2.rectangle(image, (135, 65), (160, 109), (40, 40, 40), -1)
    with pytest.raises(TopFaceNotFound):
        detect_top(image, Plane())


def test_two_similar_targets_are_ambiguous():
    image = scene()
    image[140:186, 100:191] = image[80:126, 100:191]
    with pytest.raises(TopFaceNotFound, match='Ambiguous'):
        detect_top(image, Plane())


def test_wrong_resolution_rejected():
    with pytest.raises(TopFaceNotFound, match='dimensions'):
        detect_top(scene()[::2, ::2], Plane())


def test_top_plane_uses_height_and_official_inverse_brown_deprojection():
    camera = dict(width=320, height=240, fx=300, fy=300, ppx=160, ppy=120,
                  distortion_model='distortion.inverse_brown_conrady', coefficients=[0]*5)
    transform = np.diag([1., -1., -1., 1.])
    transform[2, 3] = 1.
    plane = TopPlane(camera, {'T_base_camera': transform},
                     {'table_up_normal_base': [0, 0, 1], 'plane_d_m': 0})
    np.testing.assert_allclose(plane.points([[160, 120], [190, 120]]),
                               [[0, 0, .02961], [.097039, 0, .02961]], atol=1e-7)


def test_planner_rejection_happens_before_sdk_subprocess(tmp_path, monkeypatch):
    import plan_grasp_orientation as planner
    image = tmp_path / 'occluded.png'
    cv2.imwrite(str(image), scene(separator=False))
    output = tmp_path / 'plan'
    monkeypatch.setattr(sys, 'argv', ['planner', '--image', str(image), '--output', str(output)])
    monkeypatch.setattr(planner, 'load_plane', lambda _: Plane())
    def reject(*args):
        raise TopFaceNotFound('No top edges')
    monkeypatch.setattr(planner, 'detect_top', reject)
    monkeypatch.setattr(planner.subprocess, 'run', lambda *a, **k: pytest.fail('SDK subprocess started'))
    with pytest.raises(SystemExit) as error:
        planner.main()
    assert error.value.code == 2
    assert not (output / 'ik-input.json').exists()
    assert 'rejected' in (output / 'top-face.json').read_text()
