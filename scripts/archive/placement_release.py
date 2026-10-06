"""Small one-shot release from the known grasp, in a separate Modbus process."""
import argparse
import json
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from web_hardware import Hardware


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--execute', action='store_true', required=True)
    parser.parse_args()
    hardware = Hardware()
    try:
        hardware.connect_gripper()
        state = hardware.gripper_state()
        if state['initialized'] != 1 or state['status'] != 2 or abs(state['position']-412) > 3:
            raise RuntimeError('Expected the known stable grasp before release')
        result = hardware.gripper_step(50, lambda: None)
        state = hardware.gripper_state()
        if state['status'] != 1 or abs(state['position']-462) > 3:
            raise RuntimeError('Small opening not verified')
        print(json.dumps({'event': 'small_opening_verified', 'result': result, 'state': state}), flush=True)
    finally:
        hardware.disconnect_gripper()


if __name__ == '__main__':
    raise SystemExit('Archived completed task: cannot be replayed. Release requires a new task context.')
