"""Execute one bounded segment of the inspected elevated orientation route."""
import argparse
import hashlib
from datetime import datetime, timezone
import json
import math
import os
from pathlib import Path
import time

from sdk_runtime import sdk_runtime

ROOT = Path(__file__).resolve().parents[1]
ARCHIVED_ROUTE_DIR = ROOT / 'captures/grasp-orientation-plan'
ARCHIVED_ROUTE_HASHES = {
    'cdbdc772b2cf1caba25f54cc4fa4fa86e0671cba26dde666b345af741801da06',
    'db735defea53c911cfe41f693d29d756eb7046f5401bdd3fff81cc2ee9679f66',
}


def load_route(path, execute=False):
    document = json.loads(path.read_text())
    digest = hashlib.sha256(json.dumps(
        document, sort_keys=True, separators=(',', ':')).encode()).hexdigest()
    if execute and (path.resolve().is_relative_to(ARCHIVED_ROUTE_DIR.resolve())
                    or digest in ARCHIVED_ROUTE_HASHES):
        raise ValueError('Archived orientation route cannot be replayed; plan a new route in a new directory')
    return document['routes'][0]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--start', type=int, required=True)
    parser.add_argument('--end', type=int, required=True)
    parser.add_argument('--execute', action='store_true')
    parser.add_argument('--rate', type=float, default=.01)
    parser.add_argument('--route', type=Path, required=True,
                        help='Explicit new route; archived routes cannot be executed')
    args = parser.parse_args()
    if not math.isfinite(args.rate) or not .01 <= args.rate <= .10:
        parser.error('Joint rate must be between 0.01 and 0.10')
    route = load_route(args.route, execute=args.execute)
    rows = route['samples']
    if not 0 <= args.start < args.end < len(rows):
        raise ValueError('Invalid route interval')
    expected = rows[args.start]['joints_deg']
    target = rows[args.end]['joints_deg']
    if max(abs(a-b) for a,b in zip(expected,target)) > 5:
        raise ValueError('Segment exceeds 5 degrees')
    log_path = ROOT/'logs/runs'/('orientation-'+datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S%fZ')+'.jsonl')
    failed = False
    with sdk_runtime(), log_path.open('w') as log:
        import xCoreSDK_python as sdk
        os.chdir(ROOT/'logs/sdk')
        robot = sdk.xMateRobot()
        connected = armed = False

        def emit(event, **data):
            line = json.dumps({'time':datetime.now(timezone.utc).isoformat(),'event':event,**data},default=str)
            log.write(line+'\n'); log.flush()
            if event != 'feedback':
                print(line,flush=True)

        def call(method, *values):
            ec = {}
            result = method(*values,ec)
            if ec.get('ec') != 0:
                raise RuntimeError(f'{method.__name__}: {ec}')
            return result

        def wait_idle():
            deadline = time.monotonic()+5
            while call(robot.operationState) != sdk.OperationState.idle:
                if time.monotonic() > deadline:
                    raise RuntimeError('Stop did not reach idle')
                time.sleep(.05)

        try:
            robot.connectToRobot('192.168.0.160',''); connected = True
            if call(robot.robotInfo).type != 'XMS4-R800-B4S1A2C':
                raise RuntimeError('Unexpected robot')
            if call(robot.operateMode) != sdk.OperateMode.manual or call(robot.operationState) != sdk.OperationState.idle or call(robot.powerState) != sdk.PowerState.off:
                raise RuntimeError('Expected manual, idle, powered off')
            tool = call(robot.toolset)
            if max(abs(a-b) for a,b in zip(tool.end.trans,[0,0,.248])) > 1e-6 or max(map(abs,tool.end.rpy)) > 1e-6:
                raise RuntimeError('TCP changed')
            if max(map(abs,list(tool.ref.trans)+list(tool.ref.rpy)+list(call(robot.baseFrame)))) > 1e-6:
                raise RuntimeError('Reference frame changed')
            q = list(map(math.degrees,call(robot.jointPos)[:6]))
            if max(abs(a-b) for a,b in zip(q,expected)) > .03:
                raise RuntimeError('Robot not at segment start')
            limits_out = sdk.PyTypeVectorArrayDouble2()
            call(robot.getSoftLimit,limits_out)
            limits = limits_out.content()
            model = robot.model()
            for row in rows[args.start:args.end+1]:
                qr = list(map(math.radians,row['joints_deg']))
                if any(not lo <= v <= hi for v,(lo,hi) in zip(qr,limits)):
                    raise RuntimeError('Joint bounds')
                fk = call(model.calcFk,qr,tool)
                if max(abs(a-b) for a,b in zip(fk.trans,row['tcp_m_rad'][:3])) > .0001:
                    raise RuntimeError('Saved route FK mismatch')
            emit('plan',start_index=args.start,end_index=args.end,start_deg=q,target_deg=target,rate=args.rate)
            if args.execute:
                call(robot.setMotionControlMode,sdk.MotionControlMode.NrtCommandMode)
                call(robot.moveReset)
                cmd = sdk.MoveAbsJCommand(list(map(math.radians,target)),5,0)
                cmd.jointSpeed = args.rate
                cmd_id = sdk.PyString()
                armed = True
                call(robot.setPowerState,True)
                if call(robot.powerState) != sdk.PowerState.on:
                    raise RuntimeError('Power on unconfirmed')
                call(robot.moveAppend,[cmd],cmd_id)
                call(robot.moveStart)
                deadline = time.monotonic()+90
                while True:
                    q = list(map(math.degrees,call(robot.jointPos)[:6]))
                    pose = list(call(robot.posture,sdk.CoordinateType.endInRef))
                    if len(q) != 6 or len(pose) != 6 or not all(map(math.isfinite,q+pose)):
                        raise RuntimeError('Invalid feedback')
                    if any(not min(a,b)-.1 <= v <= max(a,b)+.1 for v,a,b in zip(q,expected,target)):
                        raise RuntimeError('Joint outside segment bounds')
                    if min(max(abs(a-b) for a,b in zip(q,row['joints_deg'])) for row in rows[args.start:args.end+1]) > .5:
                        raise RuntimeError('Joint route deviation')
                    if not (.17 <= pose[0] <= .28 and -.67 <= pose[1] <= -.56 and .105 <= pose[2] <= .15):
                        raise RuntimeError('TCP outside inspected elevated region')
                    if call(robot.operateMode) != sdk.OperateMode.manual or call(robot.powerState) != sdk.PowerState.on:
                        raise RuntimeError('Mode or power changed')
                    emit('feedback',joints_deg=q,tcp_m_rad=pose)
                    if max(abs(a-b) for a,b in zip(q,target)) < .01 and call(robot.operationState) == sdk.OperationState.idle:
                        break
                    if time.monotonic() > deadline:
                        raise RuntimeError('Motion timeout; no automatic retry')
                    time.sleep(.05)
                call(robot.stop); wait_idle()
                samples = []
                for _ in range(5):
                    time.sleep(.1)
                    q = list(map(math.degrees,call(robot.jointPos)[:6]))
                    if max(abs(a-b) for a,b in zip(q,target)) > .02:
                        raise RuntimeError('Final joint error')
                    samples.append(q)
                if max(max(axis)-min(axis) for axis in zip(*samples)) > .005:
                    raise RuntimeError('Final joints unstable')
                emit('verified',joints_deg=q,tcp_m_rad=list(call(robot.posture,sdk.CoordinateType.endInRef)))
        except (Exception,KeyboardInterrupt) as exc:
            failed = True; emit('error',message=str(exc))
        finally:
            if connected:
                if armed:
                    try:
                        call(robot.stop); wait_idle()
                        call(robot.setPowerState,False)
                        call(robot.moveReset)
                    except Exception as exc:
                        failed = True; emit('cleanup_error',message=str(exc))
                try:
                    power = call(robot.powerState)
                    emit('final_power',value=str(power))
                    if power != sdk.PowerState.off:
                        raise RuntimeError('Power off unconfirmed')
                except Exception as exc:
                    failed = True; emit('cleanup_error',message=str(exc))
                try:
                    call(robot.disconnectFromRobot); emit('disconnected')
                except Exception as exc:
                    failed = True; emit('cleanup_error',message=str(exc))
    print(log_path)
    return int(failed)


if __name__ == '__main__':
    raise SystemExit(main())
