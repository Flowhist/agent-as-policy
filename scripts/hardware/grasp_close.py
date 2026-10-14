"""Send one closing target at configured force and monitor stable contact."""

import argparse
from datetime import datetime, timezone
import json
from pathlib import Path
import time

from scripts.hardware.web_hardware import Hardware
from scripts.hardware.control_settings import load_settings, validate_force

ROOT=Path(__file__).resolve().parents[2]


def close_until_contact(hardware, emit, *, target_position=0, check=lambda: None, force_percent=None):
    force_percent = validate_force(load_settings()['gripper']['force_percent']
                                   if force_percent is None else force_percent)
    if type(target_position) is not int or not 0 <= target_position < 1000:
        raise ValueError('Closing target must be an integer in 0..999')
    check()
    state=hardware.gripper_state()
    if (not state.get('connected') or state['initialized'] != 1
            or state['status'] != 1 or state['position'] != 1000):
        raise RuntimeError('Expected initialized, idle, fully-open gripper')
    check()
    hardware.write(0x101,force_percent)
    if hardware.read(0x101) != force_percent:
        raise RuntimeError('Force setting readback failed')
    state=hardware.gripper_state()
    if (not state.get('connected') or state['initialized'] != 1
            or state['status'] != 1 or state['position'] != 1000):
        raise RuntimeError('Gripper changed before closing command')
    # This contact-seeking operation has its own full-stroke guards. Manual
    # gripper_step limits remain unchanged; no intermediate targets are sent.
    emit('closing_target',start=1000,target=target_position,force_percent=force_percent)
    deadline=time.monotonic()+15
    check()
    hardware.write(0x103,target_position)
    lowest_position=1000
    contact_seen=False
    contact_samples=[]
    empty_samples=[]
    while time.monotonic() < deadline:
        time.sleep(.15)
        check()
        state=hardware.gripper_state()
        if not state.get('connected') or state['initialized'] != 1:
            raise RuntimeError('Gripper disconnected or initialization state changed')
        actual=state['position']
        if state['status'] == 3:
            raise RuntimeError('Gripper reports object dropped')
        if not target_position-2 <= actual <= 1000 or actual > lowest_position+2:
            raise RuntimeError('Unexpected gripper displacement while closing')
        lowest_position=min(lowest_position,actual)
        if contact_seen and state['status'] != 2:
            raise RuntimeError('Contact did not remain stable')
        if state['status'] == 2:
            contact_seen=True
            emit('holding_feedback',**state)
            contact_samples.append(actual)
            contact_samples=contact_samples[-5:]
            if (len(contact_samples) == 5 and max(contact_samples)-min(contact_samples) <= 2
                    and max(contact_samples[-3:])-min(contact_samples[-3:]) <= 1):
                return {'start':1000,'target':target_position,'actual':actual,
                        'force_percent':force_percent,'holding':True}
        elif state['status'] == 1 and abs(actual-target_position) <= 2:
            empty_samples.append(actual)
            if len(empty_samples) >= 3 and max(empty_samples[-3:])-min(empty_samples[-3:]) <= 1:
                raise RuntimeError('Reached closing target without contact; pickup unverified')
        else:
            empty_samples.clear()
    raise RuntimeError('Contact timeout; no retry; serial close does not stop the gripper')


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--execute',action='store_true')
    parser.add_argument('--target-position',type=int,default=0,
                        help='Reviewed closing endpoint in raw units, 0..999 (default: 0)')
    parser.add_argument('--resident',action='store_true',help='Use resident gripper and existing control lease guards')
    parser.add_argument('--web-url',default='http://127.0.0.1:8080')
    args=parser.parse_args()
    if not 0 <= args.target_position < 1000:
        parser.error('--target-position must be in 0..999')
    path=ROOT/'logs/runs'/('grasp-close-'+datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S%fZ')+'.jsonl')
    hardware=Hardware()
    failed=False
    with path.open('x') as log:
        def emit(event,**data):
            line=json.dumps({'time':datetime.now(timezone.utc).isoformat(),'event':event,**data})
            log.write(line+'\n');log.flush();print(line,flush=True)
        hardware.on_gripper=lambda state: emit('feedback',**state)
        try:
            if not args.resident:
                hardware.connect_gripper()
            emit('plan',force_percent=load_settings()['gripper']['force_percent'],target_position=args.target_position,
                 strategy='single_target_until_stable_contact',execute=args.execute)
            if args.resident:
                from scripts.hardware.sensor_client import SensorClient
                client=SensorClient(args.web_url)
                emit('resident_snapshot',**client.gripper_snapshot())
                if args.execute:
                    with client.control():
                        result=client.action('grasp_close',target_position=args.target_position,
                                             on_feedback=lambda state: emit('feedback',**state))
                    emit('contact_verified',**result)
            elif args.execute:
                result=close_until_contact(hardware,emit,target_position=args.target_position)
                emit('contact_verified',**result)
        except (Exception,KeyboardInterrupt) as exc:
            failed=True;emit('error',message=str(exc))
        finally:
            hardware.disconnect_gripper()
            emit('resident_connection_retained' if args.resident else 'serial_closed',
                 note='Gripper remains powered; serial close is not a stop; contact alone does not prove pickup')
    print(path)
    return int(failed)


if __name__=='__main__':
    raise SystemExit(main())
