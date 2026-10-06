"""Protect the held-object observation arc, fixed joints, and box clearance."""
from pathlib import Path
import math
import sys

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'scripts'))
from holding_observe import validate_arc

Q = [-1.3, .5, -1.6, 0, -.9, 1.76]
POSE = [.26, -.58, .08, 3.116, 0, .026]
CONTEXT = {'table_normal': [0, 0, 1], 'table_d_m': .04}
DELTA = math.radians(4.8)


def test_valid_midpoint_retains_box_clearance():
    q = Q.copy(); q[0] += DELTA/2
    pose = POSE.copy(); pose[0] += .025
    validate_arc(q, pose, Q, POSE, DELTA, CONTEXT)


@pytest.mark.parametrize('index,value', [(0, Q[0]+DELTA+.01), (1, Q[1]+.001), (5, float('nan'))])
def test_unplanned_joint_motion_is_rejected(index, value):
    q = Q.copy(); q[index] = value
    with pytest.raises(RuntimeError):
        validate_arc(q, POSE, Q, POSE, DELTA, CONTEXT)


@pytest.mark.parametrize('index,value', [(0, POSE[0]+.061), (2, POSE[2]+.0003), (2, .060)])
def test_observation_cannot_leave_arc_or_lower_the_held_object(index, value):
    pose = POSE.copy(); pose[index] = value
    with pytest.raises(RuntimeError):
        validate_arc(Q, pose, Q, POSE, DELTA, CONTEXT)
