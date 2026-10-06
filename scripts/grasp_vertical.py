"""Execute a checked, fixed-XY vertical grasp stage with table-clearance guards."""

import argparse
from datetime import datetime, timezone
import json
import math
import os
from pathlib import Path
import subprocess
import time

from cartesian_step import check_pose
from sdk_runtime import sdk_runtime

ROOT = Path(__file__).resolve().parents[1]


def holding_snapshot(expected_position):
    result = subprocess.run(['python3', str(ROOT/'scripts/check_gripper.py')],
                            capture_output=True,text=True,check=True,timeout=4)
    readings = {}
    for line in result.stdout.splitlines():
        if line.startswith('{'):
            row = json.loads(line)
            if 'value' in row:
                readings[row['step']] = row['value']
    if readings.get('initialization') != 1 or readings.get('grip_status') != 2:
        raise RuntimeError('Gripper no longer reports stable contact; stop lifting')
    if abs(readings.get('position_permille',-1000)-expected_position) > 3:
        raise RuntimeError('Gripper contact position changed; stop lifting')
    return readings


def clearance(pose, context):
    return sum(a*b for a, b in zip(pose[:3], context['table_normal'])) + context['table_d_m']


def validate_column(pose, context):
    reference = context['start_tcp_m_rad']
    if len(pose) != 6 or not all(map(math.isfinite, pose)):
        raise RuntimeError('Invalid TCP feedback')
    if max(abs(a-b) for a, b in zip(pose[:2], reference[:2])) > .0005:
        raise RuntimeError('TCP left checked grasp column')
    error = max(abs(math.atan2(math.sin(a-b), math.cos(a-b)))
                for a, b in zip(pose[3:], reference[3:]))
    if error > math.radians(.2):
        raise RuntimeError('Grasp orientation changed')
    if not .019 <= clearance(pose, context) <= .172:
        raise RuntimeError('TCP outside checked table-clearance range')


def target_pose(start, context, height_mm):
    if not math.isfinite(height_mm) or not 20 <= height_mm <= 171:
        raise ValueError('Target clearance must be 20..171 mm')
    validate_column(start, context)
    target = start.copy()
    normal = context['table_normal']
    target[2] = (height_mm/1000 - context['table_d_m']
                 - normal[0]*start[0] - normal[1]*start[1])/normal[2]
    validate_column(target, context)
    return target


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--context', type=Path, required=True)
    parser.add_argument('--target-clearance-mm', type=float, required=True)
    parser.add_argument('--rate', type=float, default=.01)
    parser.add_argument('--execute', action='store_true')
    parser.add_argument('--holding-position',type=int,help='Require stable contact before each lift segment')
    args = parser.parse_args()
    if not math.isfinite(args.rate) or not .01 <= args.rate <= .10:
        parser.error('Rate must be 0.01..0.10')
    if args.holding_position is not None and not 0 <= args.holding_position <= 1000:
        parser.error('Holding position must be 0..1000')
    context = json.loads(args.context.resolve().read_text())
    if context.get('contact_reference') != 'user_confirmed_flange_to_contact_248mm':
        parser.error('Physical contact reference must be confirmed')
    target_pose(context['start_tcp_m_rad'], context, args.target_clearance_mm)
    log_path = ROOT/'logs/runs'/('grasp-vertical-'+datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S%fZ')+'.jsonl')
    failed = False
    with sdk_runtime(), log_path.open('x') as log:
        import xCoreSDK_python as sdk
        os.chdir(ROOT/'logs/sdk')
        robot = sdk.xMateRobot()
        connected = armed = False

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

        def wait_idle():
            deadline = time.monotonic()+5
            while call(robot.operationState) != sdk.OperationState.idle:
                if time.monotonic() > deadline:
                    raise RuntimeError('Stop did not reach idle')
                time.sleep(.05)

        try:
            robot.connectToRobot('192.168.0.160', ''); connected = True
            info = call(robot.robotInfo)
            if info.type != 'XMS4-R800-B4S1A2C' or info.joint_num != 6:
                raise RuntimeError('Unexpected robot')
            if (call(robot.operateMode) != sdk.OperateMode.manual
                    or call(robot.operationState) != sdk.OperationState.idle
                    or call(robot.powerState) != sdk.PowerState.off):
                raise RuntimeError('Expected manual idle powered off')
            tool = call(robot.toolset)
            if max(abs(a-b) for a, b in zip(tool.end.trans, [0,0,.248])) > 1e-6 or max(map(abs, tool.end.rpy)) > 1e-6:
                raise RuntimeError('TCP changed')
            if max(map(abs, list(tool.ref.trans)+list(tool.ref.rpy)+list(call(robot.baseFrame)))) > 1e-6:
                raise RuntimeError('Reference changed')
            bounds = sdk.PyTypeVectorArrayDouble2()
            call(robot.getSoftLimit, bounds)
            limits = bounds.content()
            if len(limits) < 6:
                raise RuntimeError('Missing joint bounds')
            start = list(call(robot.posture, sdk.CoordinateType.endInRef))
            goal = target_pose(start, context, args.target_clearance_mm)
            model = robot.model()
            initial_q = list(call(robot.jointPos)[:6])
            initial_fk = call(model.calcFk, initial_q, tool)
            previous_q, conf = initial_q, initial_fk.confData
            count = max(1, math.ceil(abs(goal[2]-start[2])/.001))
            for i in range(count+1):
                pose = start.copy(); pose[2] += (goal[2]-start[2])*i/count
                target = sdk.CartesianPosition(pose); target.confData = conf
                q = list(call(model.calcIk, target, tool))
                if len(q) != 6 or not all(map(math.isfinite, q)):
                    raise RuntimeError('Invalid IK')
                if any(not lo <= v <= hi for v, (lo,hi) in zip(q,limits)):
                    raise RuntimeError('IK joint bounds')
                if max(abs(math.degrees(a-b)) for a,b in zip(q,previous_q)) > 1:
                    raise RuntimeError('IK discontinuity')
                fk = call(model.calcFk,q,tool)
                actual = sdk.utility.postureToTransArray(list(fk.trans)+list(fk.rpy))
                wanted = sdk.utility.postureToTransArray(pose)
                if max(abs(a-b) for a,b in zip(actual,wanted)) > 1e-5:
                    raise RuntimeError('FK round-trip mismatch')
                previous_q, conf = q, fk.confData
            emit('plan', start=start, target=goal, target_clearance_mm=args.target_clearance_mm,
                 rate=args.rate, ik_samples=count+1)
            if args.execute:
                if args.holding_position is not None:
                    if goal[2] <= start[2]:
                        raise RuntimeError('Holding validation is only for lift stages')
                    emit('holding_checked',**holding_snapshot(args.holding_position))
                while abs(goal[2]-list(call(robot.posture,sdk.CoordinateType.endInRef))[2]) > .00005:
                    segment = list(call(robot.posture,sdk.CoordinateType.endInRef))
                    validate_column(segment,context)
                    # At and below the object top, restrict every command to 2 mm.
                    max_step = .002 if clearance(segment,context) < .05 else .005
                    delta = math.copysign(min(max_step,abs(goal[2]-segment[2])),goal[2]-segment[2])
                    q0 = list(call(robot.jointPos)[:6])
                    if max(abs(a-b) for a,b in zip(segment[:2],start[:2])) > .0003:
                        raise RuntimeError('XY drift accumulated')
                    if args.holding_position is not None:
                        emit('holding_checked',**holding_snapshot(args.holding_position))
                    if not armed:
                        armed = True
                        call(robot.setPowerState,True)
                    if call(robot.powerState) != sdk.PowerState.on:
                        raise RuntimeError('Power on unconfirmed')
                    call(robot.startJog,sdk.JogOptSpace.baseFrame,args.rate,abs(delta)*1000,2,delta>0)
                    deadline = time.monotonic()+20
                    while True:
                        actual = list(call(robot.posture,sdk.CoordinateType.endInRef))
                        validate_column(actual,context)
                        error = check_pose(segment,actual,2,delta*1000)
                        q = list(call(robot.jointPos)[:6])
                        if len(q) != 6 or not all(map(math.isfinite,q)):
                            raise RuntimeError('Invalid joint feedback')
                        if max(abs(math.degrees(a-b)) for a,b in zip(q,q0)) > 3:
                            raise RuntimeError('Segment joint travel exceeds 3 degrees')
                        if any(not lo <= v <= hi for v,(lo,hi) in zip(q,limits)):
                            raise RuntimeError('Joint bounds')
                        if call(robot.operateMode) != sdk.OperateMode.manual or call(robot.powerState) != sdk.PowerState.on:
                            raise RuntimeError('Mode or power changed')
                        emit('feedback',tcp_m_rad=actual,joints_rad=q)
                        if error < .00005:
                            break
                        if time.monotonic() > deadline:
                            raise RuntimeError('Target timeout; no retry')
                        time.sleep(.05)
                    call(robot.stop); wait_idle()
                    samples=[]
                    for _ in range(3):
                        time.sleep(.1)
                        actual=list(call(robot.posture,sdk.CoordinateType.endInRef))
                        if check_pose(segment,actual,2,delta*1000) > .0001:
                            raise RuntimeError('Stopped target error')
                        samples.append(actual)
                    if any(max(axis)-min(axis) > .00005 for axis in zip(*(p[:3] for p in samples))):
                        raise RuntimeError('TCP not stable')
                    emit('segment_verified',tcp_m_rad=actual,clearance_mm=clearance(actual,context)*1000)
                    if args.holding_position is not None:
                        emit('holding_checked',**holding_snapshot(args.holding_position))
                actual=list(call(robot.posture,sdk.CoordinateType.endInRef))
                emit('stage_verified',tcp_m_rad=actual,clearance_mm=clearance(actual,context)*1000)
        except (Exception,KeyboardInterrupt) as exc:
            failed=True; emit('error',message=str(exc))
        finally:
            if connected:
                if armed:
                    try:
                        call(robot.stop); wait_idle(); call(robot.setPowerState,False)
                    except Exception as exc:
                        failed=True; emit('cleanup_error',message=str(exc))
                try:
                    power=call(robot.powerState)
                    emit('final_power',value=str(power))
                    if power != sdk.PowerState.off:
                        raise RuntimeError('Power off unconfirmed')
                    emit('final_pose',tcp_m_rad=list(call(robot.posture,sdk.CoordinateType.endInRef)))
                    call(robot.disconnectFromRobot); emit('disconnected')
                except Exception as exc:
                    failed=True; emit('cleanup_error',message=str(exc))
    print(log_path)
    return int(failed)


if __name__ == '__main__':
    raise SystemExit(main())
