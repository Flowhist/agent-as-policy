from pathlib import Path
import sys

import pytest

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from grasp_vertical import clearance, target_pose, validate_column, holding_snapshot
from grasp_close import close_until_contact


CONTEXT={'start_tcp_m_rad':[.26,-.58,.13,3.1,0,0],
         'table_normal':[0,0,1],'table_d_m':.04}


def test_contact_height_preserves_xy_and_orientation():
    target=target_pose(CONTEXT['start_tcp_m_rad'],CONTEXT,20)
    assert target==[.26,-.58,-.02,3.1,0,0]
    assert clearance(target,CONTEXT)==pytest.approx(.02)


@pytest.mark.parametrize('height',[0,19,172,float('nan')])
def test_contact_floor_cannot_be_lowered(height):
    with pytest.raises(ValueError):target_pose(CONTEXT['start_tcp_m_rad'],CONTEXT,height)


@pytest.mark.parametrize('pose',[[.261,-.58,.13,3.1,0,0],[.26,-.58,-.022,3.1,0,0],
                                 [.26,-.58,.13,3.11,0,0]])
def test_feedback_rejects_column_floor_or_orientation_change(pose):
    with pytest.raises(RuntimeError):validate_column(pose,CONTEXT)


class Gripper:
    def __init__(self,contact=True):
        self.position=1000;self.holding=False;self.calls=0;self.contact=contact
    def gripper_state(self):
        return {'initialized':1,'status':2 if self.holding else 1,'position':self.position}
    def gripper_step(self,delta,check):
        assert not self.holding
        self.calls+=1;self.position+=delta
        self.holding=self.contact and self.calls==3
        return {'actual':self.position,'holding':self.holding}


def test_closure_stops_at_contact(monkeypatch):
    monkeypatch.setattr('grasp_close.time.sleep',lambda _:None)
    gripper=Gripper()
    assert close_until_contact(gripper,lambda *a,**k:None)['holding']
    assert gripper.calls==3


def test_no_contact_never_reports_pickup():
    gripper=Gripper(contact=False)
    with pytest.raises(RuntimeError,match='without verified contact'):
        close_until_contact(gripper,lambda *a,**k:None)


@pytest.mark.parametrize('status,position,valid',[(2,412,True),(3,412,False),(1,412,False),(2,420,False)])
def test_lift_checks_gripper_contact_status_and_position(monkeypatch,status,position,valid):
    from types import SimpleNamespace
    import json
    rows=[{'step':'initialization','value':1},{'step':'grip_status','value':status},
          {'step':'position_permille','value':position}]
    monkeypatch.setattr('grasp_vertical.subprocess.run',lambda *a,**k:
                        SimpleNamespace(stdout='\n'.join(json.dumps(r) for r in rows)))
    if valid:
        assert holding_snapshot(412)['grip_status']==2
    else:
        with pytest.raises(RuntimeError):holding_snapshot(412)
