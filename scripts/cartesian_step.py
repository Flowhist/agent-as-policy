"""Supervised single translation jog, at 1%, with pose and joint monitoring."""

import argparse
from datetime import datetime, timezone
import json
import math
import os
from pathlib import Path
import time

from sdk_runtime import sdk_runtime

ROOT = Path(__file__).resolve().parents[1]


def check_pose(start, actual, axis, delta):
    target = start[:3].copy()
    target[axis] += delta / 1000
    if not all(math.isfinite(x) for x in actual) or len(actual) != 6:
        raise RuntimeError('Invalid TCP feedback')
    for i in range(3):
        if not min(start[i], target[i])-.0003 <= actual[i] <= max(start[i], target[i])+.0003:
            raise RuntimeError('TCP left permitted translation corridor')
    if max(abs(math.atan2(math.sin(a-b), math.cos(a-b))) for a,b in zip(actual[3:],start[3:])) > math.radians(.2):
        raise RuntimeError('Unexpected orientation change')
    return max(abs(a-b) for a,b in zip(actual[:3],target))


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--axis',type=int,choices=(0,1,2),required=True)
    p.add_argument('--delta-mm',type=float,required=True)
    p.add_argument('--execute',action='store_true')
    args=p.parse_args()
    if not math.isfinite(args.delta_mm) or not .1 <= abs(args.delta_mm) <= 10:
        p.error('Translation must be 0.1 to 10 mm')
    folder=ROOT/'logs/runs';folder.mkdir(exist_ok=True,parents=True)
    name=folder/('cartesian-'+datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S%fZ')+'.jsonl')
    os.chdir(ROOT/'logs/sdk')
    failed=False
    with sdk_runtime(), name.open('w') as log:
        import xCoreSDK_python as sdk
        def emit(event,**data):
            text=json.dumps({'time':datetime.now(timezone.utc).isoformat(),'event':event,**data},default=str)
            log.write(text+'\n');log.flush()
            if event!='feedback':print(text,flush=True)
        def call(method,*values):
            ec={};result=method(*values,ec)
            if ec.get('ec')!=0:raise RuntimeError(f'{method.__name__}: {ec}')
            return result
        r=sdk.xMateRobot();connected=False;armed=False
        try:
            r.connectToRobot('192.168.0.160','');connected=True
            info=call(r.robotInfo)
            if info.type!='XMS4-R800-B4S1A2C' or info.joint_num!=6:raise RuntimeError('Wrong robot')
            if call(r.operateMode)!=sdk.OperateMode.manual or call(r.operationState)!=sdk.OperationState.idle:
                raise RuntimeError('Manual idle required')
            tool=call(r.toolset)
            if max(abs(a-b) for a,b in zip(tool.end.trans,[0,0,.248]))>1e-6 or max(map(abs,tool.end.rpy))>1e-6:
                raise RuntimeError('Unexpected tool transform')
            if max(map(abs,list(tool.ref.trans)+list(tool.ref.rpy)+list(call(r.baseFrame))))>1e-6:
                raise RuntimeError('Nonidentity reference/base frame')
            start=list(call(r.posture,sdk.CoordinateType.endInRef))
            joints=list(call(r.jointPos))[:6]
            limits=sdk.PyTypeVectorArrayDouble2();call(r.getSoftLimit,limits);bounds=limits.content()
            if len(bounds)<6:raise RuntimeError('Joint bounds missing')
            # Keep this commissioning tool in the inspected elevated task region.
            def workspace(pose):
                x,y,z=pose[:3]
                return .23<=x<=.30 and -.60<=y<=-.40 and .125<=z<=.15
            target=start.copy();target[args.axis]+=args.delta_mm/1000
            if not workspace(start) or not workspace(target):raise RuntimeError('Outside elevated alignment workspace')
            emit('plan',start=start,target=target,start_joints_rad=joints,tool_mass=tool.load.mass,tool_cog=tool.load.cog,rate=.01)
            if args.execute:
                time.sleep(.3)
                before=list(call(r.posture,sdk.CoordinateType.endInRef))
                if max(abs(a-b) for a,b in zip(start,before))>.0001:raise RuntimeError('Pose not stationary')
                armed=True;call(r.setPowerState,True)
                if call(r.powerState)!=sdk.PowerState.on:raise RuntimeError('Power not confirmed')
                call(r.startJog,sdk.JogOptSpace.baseFrame,.01,abs(args.delta_mm),args.axis,args.delta_mm>0)
                deadline=time.monotonic()+20;reached=False
                while time.monotonic()<deadline:
                    actual=list(call(r.posture,sdk.CoordinateType.endInRef))
                    error=check_pose(start,actual,args.axis,args.delta_mm)
                    q=list(call(r.jointPos))[:6]
                    if len(q)!=6 or not all(map(math.isfinite,q)):raise RuntimeError('Invalid joints')
                    if any(abs(a-b)>math.radians(3) for a,b in zip(q,joints)):raise RuntimeError('Joint displacement exceeds 3 degrees')
                    if any(not lo<=v<=hi for v,(lo,hi) in zip(q,bounds)):raise RuntimeError('Joint bounds exceeded')
                    if call(r.operateMode)!=sdk.OperateMode.manual or call(r.powerState)!=sdk.PowerState.on:
                        raise RuntimeError('Mode or power changed')
                    emit('feedback',pose=actual,joints=q)
                    if error<.00005:reached=True;break
                    time.sleep(.05)
                call(r.stop)
                if not reached:raise RuntimeError('Target timeout; no retry')
                samples=[]
                for _ in range(5):
                    time.sleep(.2);pose=list(call(r.posture,sdk.CoordinateType.endInRef))
                    if check_pose(start,pose,args.axis,args.delta_mm)>.0001:raise RuntimeError('Stopped target error')
                    samples.append(pose)
                if any(max(a)-min(a)>.00005 for a in zip(*(s[:3] for s in samples))):raise RuntimeError('Position not stable')
                emit('verified',actual=samples[-1],translation_mm=[(a-b)*1000 for a,b in zip(samples[-1][:3],start[:3])])
        except (Exception,KeyboardInterrupt) as exc:
            failed=True;emit('error',message=str(exc))
        finally:
            if connected:
                if armed:
                    try:
                        call(r.stop)
                        deadline=time.monotonic()+5
                        while call(r.operationState)!=sdk.OperationState.idle:
                            if time.monotonic()>=deadline:
                                raise RuntimeError('Controller did not become idle after stop')
                            time.sleep(.1)
                        emit('stop_idle_confirmed')
                    except Exception as exc:
                        failed=True;emit('cleanup_error',message=str(exc))
                actions=[(r.setPowerState,(False,))] if armed else []
                for method,values in actions:
                    try:call(method,*values)
                    except Exception as exc:failed=True;emit('cleanup_error',message=str(exc))
                try:
                    power=call(r.powerState);emit('final_power',value=str(power))
                    if armed and power!=sdk.PowerState.off:raise RuntimeError('Power off unconfirmed')
                except Exception as exc:failed=True;emit('cleanup_error',message=str(exc))
                try:call(r.disconnectFromRobot);emit('disconnected')
                except Exception as exc:failed=True;emit('cleanup_error',message=str(exc))
    print('Log:',name)
    return int(failed)


if __name__=='__main__':
    raise SystemExit(main())
