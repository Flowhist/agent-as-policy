"""Read-only SDK inverse-kinematics checks for sampled orientation paths."""

import json
import math
import os
from pathlib import Path
import sys

from scripts.hardware.sdk_runtime import sdk_runtime


def check_sampled_paths(r, sdk, data, *, check_control=lambda: None, start_joints_deg=None):
    """Check independent paths using the caller's exclusively held SDK session.

    Does not connect, disconnect, change power, or move. The caller must hold
    sdk_runtime() and keep the same session/context for subsequent execution.
    Each path starts from measured joints, or an explicit predicted start for
    batch preflight only. Execution must still verify its real starting joints.
    """
    result = {'motion_sent': False, 'paths': []}
    def call(method, *args):
        ec = {}
        value = method(*args, ec)
        if ec.get('ec') != 0:
            raise RuntimeError(f'{method.__name__}: {ec}')
        return value
    if call(r.operationState) != sdk.OperationState.idle:
        raise RuntimeError('Planning requires robot idle')
    joints = list(call(r.jointPos))[:6]
    if start_joints_deg is not None:
        if (len(start_joints_deg) != 6
                or not all(type(v) in (int, float) and math.isfinite(v) for v in start_joints_deg)):
            raise ValueError('Predicted start needs six finite joint angles')
        joints = list(map(math.radians, start_joints_deg))
    pose = list(call(r.posture, sdk.CoordinateType.endInRef))
    if max(abs(a-b) for a,b in zip(pose, data['start_tcp_m_rad'])) > .0002:
        raise RuntimeError('Robot pose differs from planning snapshot')
    tool = call(r.toolset)
    model = r.model()
    initial = call(model.calcFk, joints, tool)
    bounds = sdk.PyTypeVectorArrayDouble2()
    call(r.getSoftLimit, bounds)
    limits = bounds.content()
    result['start_joints_deg'] = list(map(math.degrees,joints))
    for path in data['paths']:
        conf = initial.confData
        previous = joints
        rows = []
        failure = None
        for i, matrix in enumerate(path['tcp_matrices_row_major']):
            check_control()
            if (not isinstance(matrix, list) or len(matrix) != 16
                    or any(type(v) not in (int, float) or not math.isfinite(v) for v in matrix)
                    or max(abs(a-b) for a,b in zip(matrix[12:], [0,0,0,1])) > 1e-8):
                raise ValueError('Expected finite row-major homogeneous matrix')
            target = sdk.CartesianPosition(sdk.utility.transArrayToPosture(matrix))
            target.confData = conf
            ec = {}
            q = list(model.calcIk(target, tool, ec))
            if ec.get('ec') != 0:
                failure = {'sample': i, 'reason': 'IK failure', 'error': ec,
                           'target_matrix': matrix, 'previous_joints_deg': list(map(math.degrees, previous)),
                           'conf_data': list(conf)}
                break
            if len(q) != 6 or not all(map(math.isfinite,q)):
                raise RuntimeError('Invalid IK output')
            fk = call(model.calcFk, q, tool)
            actual = sdk.utility.postureToTransArray(list(fk.trans)+list(fk.rpy))
            if max(abs(a-b) for a,b in zip(matrix,actual)) > 1e-5:
                failure = {'sample': i, 'reason': 'FK round-trip mismatch'}
                break
            if any(not lo <= v <= hi for v,(lo,hi) in zip(q,limits)):
                failure = {'sample': i, 'reason': 'Joint bounds'}
                break
            change = max(abs(math.degrees(a-b)) for a,b in zip(q,previous))
            rows.append({'sample': i, 'joints_deg': list(map(math.degrees,q)),
                         'max_joint_step_deg': change})
            conf = fk.confData
            previous = q
        result['paths'].append({'name': path['name'], 'complete': failure is None,
            'failure': failure, 'samples': rows,
            'min_abs_j5_deg': min((abs(row['joints_deg'][4]) for row in rows),default=None),
            'max_joint_step_deg': max((row['max_joint_step_deg'] for row in rows),default=None)})
    result['final_power'] = str(call(r.powerState))
    return result


def main(source, destination):
    source, destination = Path(source).resolve(), Path(destination).resolve()
    data = json.loads(source.read_text())
    with sdk_runtime():
        import xCoreSDK_python as sdk
        os.chdir(Path(__file__).resolve().parents[2] / 'logs/sdk')
        r = sdk.xMateRobot()
        connected = False
        def call(method, *args):
            ec = {}
            value = method(*args, ec)
            if ec.get('ec') != 0:
                raise RuntimeError(f'{method.__name__}: {ec}')
            return value
        try:
            r.connectToRobot('192.168.0.160', '')
            connected = True
            result = check_sampled_paths(r, sdk, data)
        finally:
            if connected:
                call(r.disconnectFromRobot)
    Path(destination).write_text(json.dumps(result,indent=2))
    print(json.dumps([{k:v for k,v in p.items() if k!='samples'} for p in result['paths']],indent=2))


if __name__ == '__main__':
    main(*sys.argv[1:])
