"""Progressively close at 20% force; stop issuing targets when contact is reported."""

import argparse
from datetime import datetime, timezone
import json
from pathlib import Path
import time

from web_hardware import Hardware

ROOT=Path(__file__).resolve().parents[1]


def close_until_contact(hardware, emit):
    state=hardware.gripper_state()
    if state['initialized'] != 1 or state['status'] != 1 or state['position'] != 1000:
        raise RuntimeError('Expected initialized, idle, fully-open gripper')
    for _ in range(20):
        state=hardware.gripper_state()
        if state['status'] != 1 or state['position'] < 50:
            raise RuntimeError('Unexpected state before closing step')
        result=hardware.gripper_step(-50,lambda: None)
        emit('step_verified',**result)
        if result['holding']:
            samples=[]
            for _ in range(5):
                time.sleep(.1)
                feedback=hardware.gripper_state()
                emit('holding_feedback',**feedback)
                if feedback['status'] != 2 or feedback['initialized'] != 1:
                    raise RuntimeError('Contact did not remain stable')
                samples.append(feedback['position'])
            if max(samples)-min(samples) > 2:
                raise RuntimeError('Contact position unstable')
            return result
    raise RuntimeError('Closed without verified contact; no lift authorized by feedback')


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--execute',action='store_true')
    args=parser.parse_args()
    path=ROOT/'logs/runs'/('grasp-close-'+datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S%fZ')+'.jsonl')
    hardware=Hardware()
    failed=False
    with path.open('x') as log:
        def emit(event,**data):
            line=json.dumps({'time':datetime.now(timezone.utc).isoformat(),'event':event,**data})
            log.write(line+'\n');log.flush();print(line,flush=True)
        hardware.on_gripper=lambda state: emit('feedback',**state)
        try:
            hardware.connect_gripper()
            emit('plan',force_percent=20,step_permille=-50,execute=args.execute)
            if args.execute:
                result=close_until_contact(hardware,emit)
                emit('contact_verified',**result)
        except (Exception,KeyboardInterrupt) as exc:
            failed=True;emit('error',message=str(exc))
        finally:
            hardware.disconnect_gripper()
            emit('serial_closed',note='Gripper remains powered; contact alone is not proof of successful pickup')
    print(path)
    return int(failed)


if __name__=='__main__':
    raise SystemExit(main())
