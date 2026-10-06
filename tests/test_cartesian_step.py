import math
from pathlib import Path
import subprocess
import sys

import pytest

SCRIPTS = Path(__file__).resolve().parents[1]/'scripts'
sys.path.insert(0,str(SCRIPTS))
from cartesian_step import check_expected_start, check_pose


@pytest.mark.parametrize('rate',['0','0.11','nan','inf'])
def test_rate_rejected_before_sdk(rate):
    result=subprocess.run([sys.executable,str(SCRIPTS/'cartesian_step.py'),
        '--axis','0','--delta-mm','-1','--rate',rate],capture_output=True,text=True)
    assert result.returncode==2
    assert 'Rate must' in result.stderr


def test_expected_start_detects_pose_or_joint_changes():
    start=[.26,-.58,.13,3.1,0.,0.]
    expected={'start_tcp_m_rad':start,'start_joints_deg':[0.]*6}
    check_expected_start(start,[0.]*6,expected)
    with pytest.raises(RuntimeError,match='TCP moved'):
        check_expected_start([.261]+start[1:],[0.]*6,expected)
    with pytest.raises(RuntimeError,match='Joints changed'):
        check_expected_start(start,[math.radians(.04)]+[0.]*5,expected)
    with pytest.raises(RuntimeError,match='Orientation changed'):
        check_expected_start(start[:3]+[3.11,0.,0.],[0.]*6,expected)


def test_horizontal_translation_preserves_z_and_orientation():
    start=[.26,-.58,.13,3.1,0.,0.]
    assert check_pose(start,[.257,-.58,.13,3.1,0.,0.],0,-3)<1e-10
    with pytest.raises(RuntimeError,match='corridor'):
        check_pose(start,[.257,-.58,.129,3.1,0.,0.],0,-3)
    with pytest.raises(RuntimeError,match='orientation'):
        check_pose(start,[.257,-.58,.13,3.,0.,0.],0,-3)
