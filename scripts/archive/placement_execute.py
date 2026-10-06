"""Execute the inspected placement plan once with continuous feedback guards."""
import argparse
from datetime import datetime, timezone
import hashlib
import json
import math
import os
from pathlib import Path
import subprocess
import sys
import time

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from grasp_vertical import clearance, holding_snapshot
from sdk_runtime import sdk_runtime

ROOT = Path(__file__).resolve().parents[2]
PLAN = ROOT/'captures/box-user-roi-20261006/placement-ik.json'


def angle_error(a, b):
    return abs(math.atan2(math.sin(a-b), math.cos(a-b)))


def check_feedback(pose, q, stage, limits):
    if len(pose) != 6 or len(q) != 6 or not all(map(math.isfinite, pose+q)):
        raise RuntimeError('Invalid motion feedback')
    axis = stage['axis']
    start = stage['samples'][0]['pose_m_rad']
    end = stage['samples'][-1]['pose_m_rad']
    for i in range(6):
        tolerance = .0004 if i < 3 else math.radians(.2)
        if i == axis:
            if not min(start[i], end[i])-tolerance <= pose[i] <= max(start[i], end[i])+tolerance:
                raise RuntimeError('Outside planned motion interval')
        elif (abs(pose[i]-start[i]) if i < 3 else angle_error(pose[i], start[i])) > tolerance:
            raise RuntimeError('Outside planned motion corridor')
    if any(not lo+math.radians(1) <= value <= hi-math.radians(1) for value, (lo, hi) in zip(q, limits)):
        raise RuntimeError('Joint margin violated')
    fraction = (pose[axis]-start[axis])/(end[axis]-start[axis])
    index = max(0, min(len(stage['samples'])-1, round(fraction*(len(stage['samples'])-1))))
    expected_q = stage['samples'][index]['joints_rad']
    if max(abs(a-b) for a, b in zip(q, expected_q)) > math.radians(.8):
        raise RuntimeError('Joint feedback differs from checked IK branch')
    return abs(pose[axis]-end[axis])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--execute', action='store_true')
    args = parser.parse_args()
    raw = PLAN.read_bytes()
    plan = json.loads(raw)
    if not plan.get('ik_complete') or plan.get('motion_sent'):
        raise RuntimeError('Expected complete unexecuted IK study')
    expected = [('approach_x', 0), ('align_yaw', 5), ('above_target_y', 1),
                ('above_target_x', 0), ('descend_candidate', 2), ('retreat_candidate', 2)]
    if [(s['name'], s['axis']) for s in plan['stages']] != expected:
        raise RuntimeError('Unexpected route stages')
    path = ROOT/'logs/runs'/('placement-'+datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S%fZ')+'.jsonl')
    failed = False
    with sdk_runtime(), path.open('x') as log:
        import xCoreSDK_python as sdk
        os.chdir(ROOT/'logs/sdk')
        robot = sdk.xMateRobot()
        connected = armed = released = False

        def emit(event, **data):
            line = json.dumps({'time': datetime.now(timezone.utc).isoformat(), 'event': event, **data}, default=str)
            log.write(line+'\n'); log.flush()
            if event != 'feedback':
                print(line, flush=True)

        def call(method, *values):
            ec = {}
            value = method(*values, ec)
            if ec.get('ec') != 0:
                raise RuntimeError(f'{method.__name__}: {ec}')
            return value

        def idle():
            deadline = time.monotonic()+5
            while call(robot.operationState) != sdk.OperationState.idle:
                if time.monotonic() > deadline:
                    raise RuntimeError('Stop did not reach idle')
                time.sleep(.05)

        def feedback():
            return list(call(robot.posture, sdk.CoordinateType.endInRef)), list(call(robot.jointPos)[:6])

        try:
            robot.connectToRobot('192.168.0.160', '')
            connected = True
            info = call(robot.robotInfo)
            if info.type != 'XMS4-R800-B4S1A2C' or info.joint_num != 6:
                raise RuntimeError('Unexpected robot')
            if (call(robot.operateMode) != sdk.OperateMode.manual
                    or call(robot.operationState) != sdk.OperationState.idle
                    or call(robot.powerState) != sdk.PowerState.off):
                raise RuntimeError('Expected manual idle powered-off robot')
            tool = call(robot.toolset)
            if (max(abs(a-b) for a, b in zip(tool.end.trans, [0, 0, .248])) > 1e-6
                    or max(map(abs, tool.end.rpy)) > 1e-6
                    or max(map(abs, list(tool.ref.trans)+list(tool.ref.rpy)+list(call(robot.baseFrame)))) > 1e-6):
                raise RuntimeError('Tool or reference changed')
            pose, q = feedback()
            if (max(abs(a-b) for a, b in zip(pose[:3], plan['start_tcp_m_rad'][:3])) > .0003
                    or max(angle_error(a, b) for a, b in zip(pose[3:], plan['start_tcp_m_rad'][3:])) > math.radians(.03)
                    or max(abs(a-b) for a, b in zip(q, plan['start_joints_rad'])) > math.radians(.03)):
                raise RuntimeError('Robot moved since the checked plan; replan required')
            bounds = sdk.PyTypeVectorArrayDouble2()
            call(robot.getSoftLimit, bounds)
            limits = bounds.content()
            if len(limits) != 6:
                raise RuntimeError('Missing joint bounds')
            # Audit every supplied sample before the first command.
            previous = plan['start_tcp_m_rad']
            for stage in plan['stages']:
                first = stage['samples'][0]['pose_m_rad']
                if max(abs(a-b) for a, b in zip(first, previous)) > 1e-6:
                    raise RuntimeError('Discontinuous route')
                for row in stage['samples']:
                    check_feedback(row['pose_m_rad'], row['joints_rad'], stage, limits)
                    height = clearance(row['pose_m_rad'], plan['context'])
                    if not .024 <= height <= .130:
                        raise RuntimeError('Route clearance outside reviewed range')
                    if stage['axis'] != 2 and height < .110:
                        raise RuntimeError('Horizontal/rotational motion too low')
                previous = stage['samples'][-1]['pose_m_rad']
            emit('plan_verified', rate=.10, plan_sha256=hashlib.sha256(raw).hexdigest(),
                 start=pose, user_confirmed_cabling=True, execute=args.execute)
            emit('holding_checked', **holding_snapshot(412))
            if args.execute:
                with PLAN.with_name('placement-attempt.json').open('x') as attempt:
                    json.dump({'log': str(path), 'plan_sha256': hashlib.sha256(raw).hexdigest()}, attempt)
                armed = True
                call(robot.setPowerState, True)
                if call(robot.powerState) != sdk.PowerState.on:
                    raise RuntimeError('Power on unconfirmed')
                for stage in plan['stages']:
                    name, axis = stage['name'], stage['axis']
                    pose, q = feedback()
                    check_feedback(pose, q, stage, limits)
                    first = stage['samples'][0]['pose_m_rad']
                    if abs(pose[axis]-first[axis]) > (.0003 if axis < 3 else math.radians(.03)):
                        raise RuntimeError('Unexpected stage start')
                    if not released:
                        emit('holding_checked', **holding_snapshot(412))
                    goal = stage['samples'][-1]['pose_m_rad'][axis]
                    delta = goal-pose[axis]
                    distance = abs(delta)*1000 if axis < 3 else abs(math.degrees(delta))
                    emit('stage_started', stage=name, target=goal, distance=distance)
                    call(robot.startJog, sdk.JogOptSpace.baseFrame, .10, distance, axis, delta > 0)
                    deadline = time.monotonic()+max(20, distance*.8+10)
                    tolerance = .00005 if axis < 3 else math.radians(.005)
                    while True:
                        pose, q = feedback()
                        error = check_feedback(pose, q, stage, limits)
                        if call(robot.operateMode) != sdk.OperateMode.manual or call(robot.powerState) != sdk.PowerState.on:
                            raise RuntimeError('Mode/power changed')
                        emit('feedback', stage=name, tcp_m_rad=pose, joints_rad=q)
                        if error <= tolerance:
                            break
                        if time.monotonic() > deadline:
                            raise RuntimeError('Motion timeout; no automatic retry')
                        time.sleep(.04)
                    call(robot.stop); idle()
                    for _ in range(3):
                        time.sleep(.1)
                        pose, q = feedback()
                        if check_feedback(pose, q, stage, limits) > tolerance*2:
                            raise RuntimeError('Stopped pose error')
                    emit('stage_verified', stage=name, tcp_m_rad=pose, clearance_mm=clearance(pose, plan['context'])*1000)
                    if not released:
                        emit('holding_checked', **holding_snapshot(412))
                    if name == 'descend_candidate':
                        if not .0245 <= clearance(pose, plan['context']) <= .0255:
                            raise RuntimeError('Release height mismatch')
                        result = subprocess.run(['python3', str(ROOT/'scripts/archive/placement_release.py'), '--execute'],
                                                text=True, capture_output=True, timeout=20)
                        emit('release_process', returncode=result.returncode, stdout=result.stdout, stderr=result.stderr)
                        if result.returncode != 0:
                            raise RuntimeError('Release failed; no retreat or retry')
                        released = True
                        time.sleep(.5)
                emit('placement_sequence_completed', small_opening_verified=released)
        except (Exception, KeyboardInterrupt) as exc:
            failed = True; emit('error', message=str(exc))
        finally:
            if connected:
                if armed:
                    try:
                        call(robot.stop); idle(); call(robot.setPowerState, False)
                    except Exception as exc:
                        failed = True; emit('cleanup_error', message=str(exc))
                try:
                    emit('final_state', power=str(call(robot.powerState)), operation=str(call(robot.operationState)),
                         tcp_m_rad=list(call(robot.posture, sdk.CoordinateType.endInRef)), released=released)
                    call(robot.disconnectFromRobot); emit('disconnected')
                except Exception as exc:
                    failed = True; emit('cleanup_error', message=str(exc))
    print(path)
    return int(failed)


if __name__ == '__main__':
    raise SystemExit('Archived completed task: cannot be replayed. Plan a new task from current state.')
