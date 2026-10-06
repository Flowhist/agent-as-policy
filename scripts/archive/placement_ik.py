"""Read-only continuous IK study from live feedback to an RGB layout candidate."""
import argparse
from datetime import datetime, timezone
import json
import math
import os
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from sdk_runtime import sdk_runtime

ROOT = Path(__file__).resolve().parents[2]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--layout', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    output = args.output.resolve()
    layout = json.loads(args.layout.read_text())
    choice = next(item['best'] for item in layout['options'] if item['size_m'] == [.11, .06])
    if choice is None:
        raise RuntimeError('No 110 by 60 mm candidate for the study')
    context = json.loads((ROOT/'captures/grasp-20261006T045042Z/contact-context.json').read_text())
    result = {'motion_sent': False, 'created_utc': datetime.now(timezone.utc).isoformat(),
              'status': 'kinematic_study_only_not_collision_checked', 'layout': str(args.layout.resolve()),
              'candidate': choice, 'context': context, 'stages': []}
    with sdk_runtime():
        import xCoreSDK_python as sdk
        os.chdir(ROOT/'logs/sdk')
        robot = sdk.xMateRobot()
        connected = False

        def call(method, *values):
            ec = {}
            value = method(*values, ec)
            if ec.get('ec') != 0:
                raise RuntimeError(f'{method.__name__}: {ec}')
            return value

        try:
            robot.connectToRobot('192.168.0.160', '')
            connected = True
            if (call(robot.operateMode) != sdk.OperateMode.manual
                    or call(robot.operationState) != sdk.OperationState.idle
                    or call(robot.powerState) != sdk.PowerState.off):
                raise RuntimeError('Read-only study requires manual idle powered-off robot')
            tool = call(robot.toolset)
            if (max(abs(a-b) for a, b in zip(tool.end.trans, [0, 0, .248])) > 1e-6
                    or max(map(abs, tool.end.rpy)) > 1e-6
                    or max(map(abs, list(tool.ref.trans)+list(tool.ref.rpy)+list(call(robot.baseFrame)))) > 1e-6):
                raise RuntimeError('Tool or reference changed')
            q = list(call(robot.jointPos)[:6])
            start = list(call(robot.posture, sdk.CoordinateType.endInRef))
            result['start_tcp_m_rad'], result['start_joints_rad'] = start, q
            bounds = sdk.PyTypeVectorArrayDouble2()
            call(robot.getSoftLimit, bounds)
            limits = bounds.content()
            if len(limits) != 6:
                raise RuntimeError('Missing six joint bounds')
            model = robot.model()
            initial_fk = call(model.calcFk, q, tool)
            conf = initial_fk.confData
            xy = choice['center_base_xy_m']
            normal, d = context['table_normal'], context['table_d_m']
            release_z = (.025-d-normal[0]*xy[0]-normal[1]*xy[1])/normal[2]
            stages = [('approach_x', 0, .180), ('align_yaw', 5, layout['yaw_rad']),
                      ('above_target_y', 1, xy[1]), ('above_target_x', 0, xy[0]),
                      ('descend_candidate', 2, release_z), ('retreat_candidate', 2, start[2])]
            previous_pose = start.copy()
            for name, axis, goal in stages:
                count = max(1, math.ceil(abs(goal-previous_pose[axis])/(math.radians(.25) if axis == 5 else .001)))
                rows = []
                for index in range(count+1):
                    pose = previous_pose.copy()
                    pose[axis] += (goal-previous_pose[axis])*index/count
                    target = sdk.CartesianPosition(pose)
                    target.confData = conf
                    current_q = list(call(model.calcIk, target, tool))
                    if len(current_q) != 6 or not all(map(math.isfinite, current_q)):
                        raise RuntimeError('Invalid IK')
                    if any(not lo+math.radians(2) <= value <= hi-math.radians(2)
                           for value, (lo, hi) in zip(current_q, limits)):
                        raise RuntimeError(f'{name} sample {index}: less than 2 degrees joint margin')
                    if max(abs(a-b) for a, b in zip(q, current_q)) > math.radians(1):
                        raise RuntimeError(f'{name}: IK discontinuity')
                    if abs(current_q[4]) < math.radians(10):
                        raise RuntimeError('Near wrist singularity')
                    fk = call(model.calcFk, current_q, tool)
                    actual = sdk.utility.postureToTransArray(list(fk.trans)+list(fk.rpy))
                    wanted = sdk.utility.postureToTransArray(pose)
                    if max(abs(a-b) for a, b in zip(actual, wanted)) > 1e-5:
                        raise RuntimeError('FK round-trip mismatch')
                    rows.append({'pose_m_rad': pose, 'joints_rad': current_q})
                    q, conf = current_q, fk.confData
                result['stages'].append({'name': name, 'axis': axis, 'target': goal, 'samples': rows})
                previous_pose[axis] = goal
            result['ik_complete'] = True
            result['minimum_joint_margin_deg'] = min(math.degrees(min(v-lo, hi-v))
                for stage in result['stages'] for row in stage['samples']
                for v, (lo, hi) in zip(row['joints_rad'], limits))
        except Exception as exc:
            result['ik_complete'] = False
            result['error'] = str(exc)
        finally:
            if connected:
                result['final_power'] = str(call(robot.powerState))
                call(robot.disconnectFromRobot)
    with output.open('x') as stream:
        json.dump(result, stream, indent=2)
    print(json.dumps({k: v for k, v in result.items() if k not in ('stages', 'context')}, indent=2))
    return int(not result['ik_complete'])


if __name__ == '__main__':
    raise SystemExit('Archived scene-specific planner: cannot be replayed. See scripts/README.md.')
