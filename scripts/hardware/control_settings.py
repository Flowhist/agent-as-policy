"""Shared robot defaults, validated on every read; no device imports."""

import fcntl
import hashlib
import json
import math
import os
from pathlib import Path
import tempfile

import yaml

ROOT = Path(__file__).resolve().parents[2]
PATH = ROOT / 'config/control.yaml'


def validate_rate(value):
    if type(value) not in (int, float) or not math.isfinite(value) or not .01 <= value <= 1:
        raise ValueError('SDK rate must be 0.01..1.0')
    return value


def validate_force(value):
    if type(value) is not int or not 20 <= value <= 100:
        raise ValueError('Gripper force must be an integer in 20..100 percent')
    return value


def validate_settings(value):
    if (not isinstance(value, dict) or set(value) != {'robot', 'gripper'}
            or not isinstance(value['robot'], dict) or set(value['robot']) != {'sdk_rate'}
            or not isinstance(value['gripper'], dict) or set(value['gripper']) != {'force_percent'}):
        raise ValueError('Expected robot.sdk_rate and gripper.force_percent only')
    return {'robot': {'sdk_rate': validate_rate(value['robot']['sdk_rate'])},
            'gripper': {'force_percent': validate_force(value['gripper']['force_percent'])}}


def read_settings(path=PATH):
    raw = Path(path).read_bytes()
    return {'values': validate_settings(yaml.safe_load(raw)),
            'revision': hashlib.sha256(raw).hexdigest()}


def load_settings():
    return read_settings()['values']


def save_settings(values, revision, path=PATH):
    values = validate_settings(values)
    path = Path(path)
    lock_path = ROOT / 'logs/control-settings.lock'
    lock_path.parent.mkdir(parents=True, exist_ok=True)
    with lock_path.open('a') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX)
        if read_settings(path)['revision'] != revision:
            raise ValueError('Settings changed; reload before saving')
        name = None
        try:
            with tempfile.NamedTemporaryFile(mode='w', dir=path.parent, delete=False) as out:
                name = out.name
                os.chmod(name, path.stat().st_mode & 0o777)
                yaml.safe_dump(values, out, sort_keys=False)
                out.flush()
                os.fsync(out.fileno())
            os.replace(name, path)
        finally:
            if name and os.path.exists(name):
                os.unlink(name)
        return read_settings(path)


if __name__ == '__main__':
    print(json.dumps(read_settings(), indent=2))
