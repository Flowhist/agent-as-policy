"""Execution corridor rejection without robot or gripper access."""
import math
from pathlib import Path
import sys

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'scripts'))
from archive.placement_execute import check_feedback


@pytest.fixture
def route():
    samples = [{'pose_m_rad': [.18-i*.001, -.56, .08, 3.11, 0., -.1],
                'joints_rad': [-1.2, .5, -1.6, 0., -.9, 1.7]} for i in range(11)]
    return {'axis': 0, 'samples': samples}, [[-2.8, 2.8]]*6


def test_corridor_accepts_planned_feedback(route):
    stage, limits = route
    row = stage['samples'][5]
    assert check_feedback(row['pose_m_rad'], row['joints_rad'], stage, limits) == pytest.approx(.005)


@pytest.mark.parametrize('kind', ['overshoot', 'sideways', 'rotation', 'branch', 'nonfinite'])
def test_unplanned_motion_is_rejected(route, kind):
    stage, limits = route
    pose = stage['samples'][5]['pose_m_rad'].copy()
    q = stage['samples'][5]['joints_rad'].copy()
    if kind == 'overshoot':
        pose[0] = .169
    elif kind == 'sideways':
        pose[1] += .001
    elif kind == 'rotation':
        pose[5] += math.radians(1)
    elif kind == 'branch':
        q[0] += math.radians(2)
    else:
        pose[0] = float('nan')
    with pytest.raises(RuntimeError):
        check_feedback(pose, q, stage, limits)
