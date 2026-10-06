"""Plan and execute a new J1-only observation arc while retaining the grasp."""
import argparse
from datetime import datetime, timezone
import json
import math
import os
from pathlib import Path
import time

from grasp_vertical import clearance, holding_snapshot
from sdk_runtime import sdk_runtime

ROOT = Path(__file__).resolve().parents[1]


def validate_arc(q, pose, baseline_q, baseline_pose, delta, context):
    if len(q) != 6 or len(pose) != 6 or not all(map(math.isfinite, q+pose)):
        raise RuntimeError('Invalid observation feedback')
    if not min(baseline_q[0], baseline_q[0]+delta)-math.radians(.03) <= q[0] <= max(baseline_q[0], baseline_q[0]+delta)+math.radians(.03):
        raise RuntimeError('J1 outside observation arc')
    if max(abs(a-b) for a, b in zip(q[1:], baseline_q[1:])) > math.radians(.03):
        raise RuntimeError('Another joint moved during J1 observation')
    if math.dist(pose[:2], baseline_pose[:2]) > .060:
        raise RuntimeError('TCP moved beyond observation envelope')
    if abs(pose[2]-baseline_pose[2]) > .0002:
        raise RuntimeError('Observation changed TCP height')
    # Includes 30 mm below TCP and 30 mm above the 50 mm box rim.
    if not .110 <= clearance(pose, context) <= .130:
        raise RuntimeError('Insufficient clearance for held-object observation')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--directory', type=Path, required=True)
    parser.add_argument('--phase', choices=['plan', 'out', 'return'], required=True)
    parser.add_argument('--delta-deg', type=float, default=4.8)
    parser.add_argument('--rate', type=float, default=.10)
    parser.add_argument('--execute', action='store_true')
    args = parser.parse_args()
    if not math.isfinite(args.rate) or not .01 <= args.rate <= .10:
        parser.error('Rate must be 0.01..0.10')
    if not math.isfinite(args.delta_deg) or not .01 <= abs(args.delta_deg) <= 5:
        parser.error('Observation arc must be 0.01..5 degrees')
    if args.phase == 'plan' and args.execute:
        parser.error('Planning never executes')
    directory = args.directory.resolve()
    plan_path = directory/'plan.json'
    if args.phase == 'plan':
        directory.mkdir(exist_ok=False, parents=True)
        context = json.loads((ROOT/'captures/grasp-20261006T045042Z/contact-context.json').read_text())
        plan = None
    else:
        plan = json.loads(plan_path.read_text())
        if plan['purpose'] != 'new_holding_J1_observation_50mm_rim':
            parser.error('Wrong route purpose')
        context = plan['context']
    log_path = ROOT/'logs/runs'/('holding-observe-'+datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S%fZ')+'.jsonl')
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
                raise RuntimeError('Wrong robot')
            if (call(robot.operateMode) != sdk.OperateMode.manual
                    or call(robot.operationState) != sdk.OperationState.idle
                    or call(robot.powerState) != sdk.PowerState.off):
                raise RuntimeError('Expected manual idle powered-off robot')
            tool = call(robot.toolset)
            if (max(abs(a-b) for a, b in zip(tool.end.trans, [0, 0, .248])) > 1e-6
                    or max(map(abs, tool.end.rpy)) > 1e-6
                    or max(map(abs, list(tool.ref.trans)+list(tool.ref.rpy)+list(call(robot.baseFrame)))) > 1e-6):
                raise RuntimeError('Tool/base/reference changed')
            bounds = sdk.PyTypeVectorArrayDouble2(); call(robot.getSoftLimit, bounds)
            limits = bounds.content()
            if len(limits) < 6:
                raise RuntimeError('Missing joint limits')
            q = list(call(robot.jointPos)[:6])
            pose = list(call(robot.posture, sdk.CoordinateType.endInRef))
            model = robot.model()
            if plan is None:
                baseline_q, baseline_pose = q, pose
                delta = math.radians(args.delta_deg)
            else:
                baseline_q, baseline_pose = plan['baseline_joints_rad'], plan['baseline_tcp_m_rad']
                delta = plan['delta_rad']
            samples = []
            for index in range(51):
                sample_q = baseline_q.copy(); sample_q[0] += delta*index/50
                if any(not lo <= v <= hi for v, (lo, hi) in zip(sample_q, limits)):
                    raise RuntimeError('Observation exceeds joint bounds')
                fk = call(model.calcFk, sample_q, tool)
                sample_pose = list(fk.trans)+list(fk.rpy)
                validate_arc(sample_q, sample_pose, baseline_q, baseline_pose, delta, context)
                samples.append({'joints_rad': sample_q, 'tcp_m_rad': sample_pose,
                                'clearance_mm': clearance(sample_pose, context)*1000})
            if plan is None:
                plan = {'purpose': 'new_holding_J1_observation_50mm_rim', 'created_utc': datetime.now(timezone.utc).isoformat(),
                        'motion_sent': False, 'baseline_joints_rad': baseline_q, 'baseline_tcp_m_rad': baseline_pose,
                        'delta_rad': delta, 'context': context, 'holding_position': 412, 'samples': samples,
                        'limitation': 'FK and bounded observation arc; not a horizontal transport route'}
                with plan_path.open('x') as stream:
                    json.dump(plan, stream, indent=2)
                emit('plan', baseline_tcp_m_rad=baseline_pose, baseline_joints_deg=list(map(math.degrees, baseline_q)),
                     observation_tcp_m_rad=samples[-1]['tcp_m_rad'], delta_deg=math.degrees(delta),
                     min_clearance_mm=min(row['clearance_mm'] for row in samples), samples=len(samples))
            else:
                expected = baseline_q.copy()
                goal = baseline_q.copy()
                if args.phase == 'out':
                    goal[0] += delta
                else:
                    expected[0] += delta
                if max(abs(a-b) for a, b in zip(q, expected)) > math.radians(.03):
                    raise RuntimeError('Robot not at newly planned phase start')
                validate_arc(q, pose, baseline_q, baseline_pose, delta, context)
                emit('plan', phase=args.phase, start_deg=list(map(math.degrees, q)), target_deg=list(map(math.degrees, goal)), rate=args.rate)
                if args.execute:
                    # Exclusive attempt record disallows replay/retry, including an uncertain failure.
                    with (directory/(args.phase+'-attempt.json')).open('x') as stream:
                        json.dump({'log': str(log_path), 'time': datetime.now(timezone.utc).isoformat()}, stream)
                    emit('holding_checked', **holding_snapshot(plan['holding_position']))
                    while abs(q[0]-goal[0]) > math.radians(.005):
                        segment_q = q.copy()
                        move_deg = math.copysign(min(1.2, abs(math.degrees(goal[0]-q[0]))), goal[0]-q[0])
                        segment_goal = q.copy(); segment_goal[0] += math.radians(move_deg)
                        emit('holding_checked', **holding_snapshot(plan['holding_position']))
                        if not armed:
                            armed = True; call(robot.setPowerState, True)
                        if call(robot.powerState) != sdk.PowerState.on:
                            raise RuntimeError('Power on unconfirmed')
                        call(robot.startJog, sdk.JogOptSpace.jointSpace, args.rate, abs(move_deg), 0, move_deg > 0)
                        deadline = time.monotonic()+15
                        while True:
                            q = list(call(robot.jointPos)[:6])
                            pose = list(call(robot.posture, sdk.CoordinateType.endInRef))
                            validate_arc(q, pose, baseline_q, baseline_pose, delta, context)
                            if not min(segment_q[0], segment_goal[0])-.0005 <= q[0] <= max(segment_q[0], segment_goal[0])+.0005:
                                raise RuntimeError('J1 outside segment')
                            fk = call(model.calcFk, q, tool)
                            if max(abs(a-b) for a, b in zip(fk.trans, pose[:3])) > .0003:
                                raise RuntimeError('TCP and joint FK disagree')
                            if any(not lo <= v <= hi for v, (lo, hi) in zip(q, limits)):
                                raise RuntimeError('Runtime joint bounds')
                            if call(robot.operateMode) != sdk.OperateMode.manual or call(robot.powerState) != sdk.PowerState.on:
                                raise RuntimeError('Mode/power changed')
                            emit('feedback', joints_rad=q, tcp_m_rad=pose)
                            if max(abs(a-b) for a, b in zip(q, segment_goal)) < math.radians(.005):
                                break
                            if time.monotonic() > deadline:
                                raise RuntimeError('Observation timeout; no retry')
                            time.sleep(.05)
                        call(robot.stop); wait_idle()
                        stable = []
                        for _ in range(3):
                            time.sleep(.1)
                            q = list(call(robot.jointPos)[:6])
                            if max(abs(a-b) for a, b in zip(q, segment_goal)) > math.radians(.01):
                                raise RuntimeError('Stopped J1 target error')
                            stable.append(q)
                        if any(max(axis)-min(axis) > math.radians(.005) for axis in zip(*stable)):
                            raise RuntimeError('Stopped joints not stable')
                        emit('holding_checked', **holding_snapshot(plan['holding_position']))
                        emit('segment_verified', joints_deg=list(map(math.degrees, q)), tcp_m_rad=list(call(robot.posture, sdk.CoordinateType.endInRef)))
                    emit('phase_verified', phase=args.phase, joints_deg=list(map(math.degrees, q)), tcp_m_rad=list(call(robot.posture, sdk.CoordinateType.endInRef)))
        except (Exception, KeyboardInterrupt) as exc:
            failed = True; emit('error', message=str(exc))
        finally:
            if connected:
                if armed:
                    try:
                        call(robot.stop); wait_idle(); call(robot.setPowerState, False)
                    except Exception as exc:
                        failed = True; emit('cleanup_error', message=str(exc))
                try:
                    power = call(robot.powerState); emit('final_power', value=str(power))
                    if power != sdk.PowerState.off:
                        raise RuntimeError('Power off unconfirmed')
                    emit('final_pose', tcp_m_rad=list(call(robot.posture, sdk.CoordinateType.endInRef)))
                    call(robot.disconnectFromRobot); emit('disconnected')
                except Exception as exc:
                    failed = True; emit('cleanup_error', message=str(exc))
    print(log_path)
    return int(failed)


if __name__ == '__main__':
    raise SystemExit(main())
