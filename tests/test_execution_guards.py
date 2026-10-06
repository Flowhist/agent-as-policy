"""Offline checks: reject unsafe entry points before connecting to hardware."""
import json
from pathlib import Path
import subprocess
import sys

import pytest

SCRIPTS = Path(__file__).resolve().parents[1] / 'scripts'
sys.path.insert(0, str(SCRIPTS))
import orientation_joint_step as orientation


@pytest.mark.parametrize('vision', ['cv2', 'pyrealsense2'])
@pytest.mark.parametrize('sdk_first', [False, True])
def test_native_library_import_order_is_guarded(vision, sdk_first):
    # Separate interpreters avoid loading any native library or polluting pytest.
    first, second = ('xCoreSDK_python', vision) if sdk_first else (vision, 'xCoreSDK_python')
    code = f'''
import sys, types
sys.path.insert(0, {str(SCRIPTS)!r})
import sdk_runtime
sys.modules[{first!r}] = types.ModuleType({first!r})
try:
    __import__({second!r})
except RuntimeError as exc:
    assert 'separate processes' in str(exc)
else:
    raise AssertionError('Mixed native imports accepted')
'''
    subprocess.run([sys.executable, '-c', code], check=True)


def test_sdk_entry_rejects_loaded_vision(monkeypatch):
    import sdk_runtime
    monkeypatch.setitem(sys.modules, 'cv2', object())
    with pytest.raises(RuntimeError, match='separate processes'):
        with sdk_runtime.sdk_runtime():
            pytest.fail('SDK context entered')


def test_route_is_required_before_sdk_start():
    result = subprocess.run(
        [sys.executable, str(SCRIPTS / 'orientation_joint_step.py'),
         '--start', '0', '--end', '1', '--execute'], capture_output=True, text=True)
    assert result.returncode == 2
    assert '--route' in result.stderr


def test_archived_directory_rejected_but_readable(tmp_path, monkeypatch):
    monkeypatch.setattr(orientation, 'ARCHIVED_ROUTE_DIR', tmp_path)
    path = tmp_path / 'route.json'
    path.write_text(json.dumps({'routes': [{'samples': []}]}))
    assert orientation.load_route(path) == {'samples': []}
    with pytest.raises(ValueError, match='cannot be replayed'):
        orientation.load_route(path, execute=True)


def test_copied_archive_rejected_after_reformatting(tmp_path, monkeypatch):
    document = {'routes': [{'samples': []}]}
    digest = orientation.hashlib.sha256(json.dumps(
        document, sort_keys=True, separators=(',', ':')).encode()).hexdigest()
    monkeypatch.setattr(orientation, 'ARCHIVED_ROUTE_HASHES', {digest})
    path = tmp_path / 'renamed.json'
    path.write_text(json.dumps(document, indent=4))
    with pytest.raises(ValueError, match='cannot be replayed'):
        orientation.load_route(path, execute=True)


def test_new_route_can_be_loaded(tmp_path):
    path = tmp_path / 'new.json'
    path.write_text(json.dumps({'routes': [{'samples': []}]}))
    assert orientation.load_route(path, execute=True) == {'samples': []}
