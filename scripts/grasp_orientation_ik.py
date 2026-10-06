"""Read-only SDK inverse-kinematics checks for sampled orientation paths."""

import json
import math
import os
from pathlib import Path
import sys

from sdk_runtime import sdk_runtime


def main(source, destination):
    data = json.loads(Path(source).read_text())
    result = {'motion_sent': False, 'paths': []}
    with sdk_runtime():
        import xCoreSDK_python as sdk
        os.chdir(Path(__file__).resolve().parents[1] / 'logs/sdk')
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
            if call(r.operationState) != sdk.OperationState.idle or call(r.powerState) != sdk.PowerState.off:
                raise RuntimeError('Planning requires robot idle and powered off')
            joints = list(call(r.jointPos))[:6]
            pose = list(call(r.posture, sdk.CoordinateType.endInRef))
            if max(abs(a-b) for a,b in zip(pose, data['start_tcp_m_rad'])) > .0002:
                raise RuntimeError('Robot pose differs from planning snapshot')
            tool = call(r.toolset)
            if max(abs(a-b) for a,b in zip(tool.end.trans, [0,0,.248])) > 1e-6 or max(map(abs,tool.end.rpy)) > 1e-6:
                raise RuntimeError('Unexpected TCP transform')
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
                    target = sdk.CartesianPosition(sdk.utility.transArrayToPosture(matrix))
                    target.confData = conf
                    ec = {}
                    q = list(model.calcIk(target, tool, ec))
                    if ec.get('ec') != 0:
                        failure = {'sample': i, 'reason': 'IK failure', 'error': ec}
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
                    if change > 10:
                        failure = {'sample': i, 'reason': 'IK branch discontinuity', 'joint_step_deg': change}
                        break
                    rows.append({'sample': i, 'joints_deg': list(map(math.degrees,q)),
                                 'max_joint_step_deg': change})
                    conf = fk.confData
                    previous = q
                result['paths'].append({'name': path['name'], 'complete': failure is None,
                    'failure': failure, 'samples': rows,
                    'min_abs_j5_deg': min((abs(row['joints_deg'][4]) for row in rows),default=None),
                    'max_joint_step_deg': max((row['max_joint_step_deg'] for row in rows),default=None)})
            result['final_power'] = str(call(r.powerState))
        finally:
            if connected:
                call(r.disconnectFromRobot)
    Path(destination).write_text(json.dumps(result,indent=2))
    print(json.dumps([{k:v for k,v in p.items() if k!='samples'} for p in result['paths']],indent=2))


if __name__ == '__main__':
    main(*sys.argv[1:])
