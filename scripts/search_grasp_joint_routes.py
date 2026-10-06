"""Read-only search for grasp IK branches and sampled joint-space routes."""
import json
import math
import os
from pathlib import Path

from sdk_runtime import sdk_runtime

ROOT = Path(__file__).resolve().parents[1]


def main():
    folder = ROOT / 'captures/grasp-orientation-plan'
    data = json.loads((folder / 'ik-input.json').read_text())
    report = {'motion_sent': False, 'routes': [], 'attempts': 0}
    with sdk_runtime():
        import xCoreSDK_python as sdk
        os.chdir(ROOT / 'logs/sdk')
        robot = sdk.xMateRobot()

        def call(method, *args):
            ec = {}
            result = method(*args, ec)
            if ec.get('ec') != 0:
                raise RuntimeError(str(ec))
            return result

        robot.connectToRobot('192.168.0.160', '')
        try:
            if call(robot.operationState) != sdk.OperationState.idle or call(robot.powerState) != sdk.PowerState.off:
                raise RuntimeError('Expected idle and powered off')
            q0 = list(call(robot.jointPos))[:6]
            pose = list(call(robot.posture, sdk.CoordinateType.endInRef))
            if max(abs(a-b) for a,b in zip(pose,data['start_tcp_m_rad'])) > .0002:
                raise RuntimeError('Start pose changed')
            tool = call(robot.toolset)
            model = robot.model()
            limits_out = sdk.PyTypeVectorArrayDouble2()
            call(robot.getSoftLimit, limits_out)
            limits = limits_out.content()
            report['start_joints_deg'] = list(map(math.degrees,q0))
            seen = set()
            for path in data['paths']:
                if '_lift_' in path['name']:
                    continue
                for j4 in (-100, -50, 0, 50, 100):
                    for j5 in (-90, -45, 15, 45, 90):
                        for j6 in (-100, 0, 100):
                            seed = q0[:3] + list(map(math.radians,(j4,j5,j6)))
                            target = sdk.CartesianPosition(sdk.utility.transArrayToPosture(path['tcp_matrices_row_major'][-1]))
                            target.confData = call(model.calcFk, seed, tool).confData
                            ec = {}
                            q = list(model.calcIk(target, tool, ec))
                            report['attempts'] += 1
                            if ec.get('ec') != 0 or len(q) != 6 or not all(map(math.isfinite,q)):
                                continue
                            if any(not lo <= v <= hi for v,(lo,hi) in zip(q,limits)):
                                continue
                            key = tuple(round(math.degrees(v),1) for v in q)
                            if key in seen:
                                continue
                            seen.add(key)
                            rows = []
                            count = math.ceil(max(abs(math.degrees(a-b)) for a,b in zip(q,q0))/.25)
                            for i in range(count+1):
                                qi = [a+(b-a)*i/count for a,b in zip(q0,q)]
                                fk = call(model.calcFk, qi, tool)
                                rows.append({'joints_deg':list(map(math.degrees,qi)), 'tcp_m_rad':list(fk.trans)+list(fk.rpy)})
                            report['routes'].append({'name':path['name'], 'goal_joints_deg':list(key),
                                'min_tcp_z_m':min(row['tcp_m_rad'][2] for row in rows),
                                'min_abs_j5_deg':min(abs(row['joints_deg'][4]) for row in rows),
                                'max_joint_travel_deg':max(abs(math.degrees(a-b)) for a,b in zip(q,q0)),
                                'samples':rows})
            report['final_power'] = str(call(robot.powerState))
        finally:
            call(robot.disconnectFromRobot)
    (folder/'joint-route-search.json').write_text(json.dumps(report,indent=2))
    print(json.dumps({**report,'routes':[{k:v for k,v in r.items() if k!='samples'} for r in report['routes']]},indent=2))


if __name__ == '__main__':
    main()
