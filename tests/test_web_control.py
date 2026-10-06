import sys
import time
import importlib.machinery
import os
import subprocess
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
import web_control
import sdk_runtime as sdk_module
from web_control import Camera, Controller, validate
from web_hardware import Hardware, SimHardware, jog_timeout
from sdk_runtime import sdk_runtime


def wait_for(predicate, timeout=3):
    deadline = time.monotonic() + timeout
    while not predicate():
        assert time.monotonic() < deadline, "Timed out waiting for control state"
        time.sleep(0.02)


@pytest.fixture
def controller(tmp_path):
    c = Controller(SimHardware(), tmp_path, simulate=True)
    yield c
    c.close()


@pytest.mark.parametrize("action,args", [
    ("jog", {"joint": 0, "delta": 0.1}),
    ("jog", {"joint": True, "delta": 0.1}),
    ("jog", {"joint": 1, "delta": 50.01}),
    ("jog", {"joint": 6, "delta": -50.01}),
    ("jog", {"joint": 1, "delta": float("nan")}),
    ("jog", {"joint": 1, "delta": 0.1, "rate": 1}),
    ("gripper_step", {"delta": 501}),
    ("gripper_step", {"delta": True}),
    ("drag", {"enabled": "false"}),
    ("joint_target", {"joint": 1, "target_deg": float("nan")}),
    ("joint_target", {"joint": 1, "target_deg": 10, "rate": 0}),
    ("gripper_target", {"target": 1001}),
    ("gripper_target", {"target": True}),
    ("power", {}),
])
def test_rejects_unbounded_or_ambiguous_commands(action, args):
    with pytest.raises(ValueError):
        validate(action, args)


@pytest.mark.parametrize("joint", range(1, 7))
def test_requested_large_steps_are_allowed(joint):
    for step in (0.1, 0.5, 1, 5, 10, 25, 50):
        for direction in (-1, 1):
            assert validate("jog", {"joint": joint, "delta": direction * step})["delta"] == direction * step
    for raw in (1, 5, 10, 50, 100, 250, 500):
        assert validate("gripper_step", {"delta": raw})["delta"] == raw


def test_large_jog_timeout_scales_with_travel_and_rate():
    assert jog_timeout(50, 0.01) == 210
    assert jog_timeout(-50, 0.01) == 210
    assert jog_timeout(50, 0.10) == 30


def test_absolute_joint_and_gripper_targets(controller):
    c = controller
    c.claim("a")
    for action in ("connect_robot", "connect_gripper"):
        c.submit("a", action, {})
        wait_for(lambda: not c.busy)
    c.submit("a", "joint_target", {"joint": 1, "target_deg": -20.0, "rate": 0.01})
    wait_for(lambda: not c.busy)
    assert c.hw.joints[0] == pytest.approx(-20)
    c.submit("a", "gripper_target", {"target": 650})
    wait_for(lambda: not c.busy)
    assert c.hw.position == 650


def test_absolute_targets_check_latest_position_and_bounds():
    h = SimHardware()
    h.connect_robot()
    h.connect_gripper()
    with pytest.raises(RuntimeError, match="bounds"):
        h.jog(1, None, 0.01, lambda: None, target_deg=60)
    h.joints[4] = 160
    with pytest.raises(RuntimeError, match="bounds"):
        h.jog(5, None, 0.01, lambda: None, target_deg=170)
    with pytest.raises(RuntimeError, match="50%"):
        h.gripper_step(None, lambda: None, target=100)
    assert h.position == 1000


@pytest.mark.parametrize('start_status,target,feedback,holding', [
    (1, 200, [(0, 450), (2, 350), (2, 350), (2, 350)], True),
    (2, 800, [(2, 500), (0, 650), (1, 800), (1, 800), (1, 800)], False),
    (2, 200, [(2, 500), (2, 500), (2, 500)], True),
])
def test_gripper_contact_and_release(monkeypatch, start_status, target, feedback, holding):
    h = Hardware.__new__(Hardware)
    samples = iter([(start_status, 500)] + feedback)
    def state():
        status, pos = next(samples)
        return dict(connected=True, initialized=1, status=status, position=pos)
    h.gripper_state = state
    writes = []
    h.write = lambda address, value: writes.append((address, value))
    h.read = lambda address: 20
    monkeypatch.setattr('web_hardware.time.sleep', lambda _: None)
    result = h.gripper_step(None, lambda: None, target=target)
    assert result['holding'] is holding
    assert result['actual'] == feedback[-1][1]
    assert writes == [(0x101, 20), (0x103, target)]


@pytest.mark.parametrize('initialized,status,message', [
    (1, 3, 'dropped'), (0, 2, 'initialization'),
])
def test_gripper_contact_does_not_hide_faults(monkeypatch, initialized, status, message):
    h = Hardware.__new__(Hardware)
    samples = iter([dict(connected=True, initialized=1, status=1, position=500),
                    dict(connected=True, initialized=initialized, status=status, position=350)])
    h.gripper_state = lambda: next(samples)
    h.write = lambda *_: None
    h.read = lambda _: 20
    monkeypatch.setattr('web_hardware.time.sleep', lambda _: None)
    with pytest.raises(RuntimeError, match=message):
        h.gripper_step(None, lambda: None, target=200)


def test_single_owner_and_no_action_queue(controller):
    c = controller
    c.claim("a")
    with pytest.raises(ValueError): c.claim("b")
    with pytest.raises(RuntimeError): c.submit("b", "connect_robot", {})
    c.submit("a", "connect_robot", {})
    wait_for(lambda: not c.busy)
    c.submit("a", "jog", {"joint": 1, "delta": 0.1})
    with pytest.raises(ValueError): c.submit("a", "jog", {"joint": 1, "delta": 0.1})
    wait_for(lambda: not c.busy)
    assert c.hw.joints[0] == pytest.approx(-26.38)


def test_stop_cancels_running_command_and_revokes_control(controller):
    c = controller
    c.claim("a")
    c.submit("a", "connect_robot", {})
    wait_for(lambda: not c.busy)
    start = c.hw.joints.copy()
    c.submit("a", "jog", {"joint": 1, "delta": 0.1})
    time.sleep(0.1)
    c.stop()
    wait_for(lambda: not c.busy and not c.stopping.is_set())
    assert c.hw.joints == start
    assert c.owner is None
    with pytest.raises(RuntimeError): c.submit("a", "jog", {"joint": 1, "delta": 0.1})


def test_lost_heartbeat_exits_drag(controller):
    c = controller
    c.claim("a")
    c.submit("a", "connect_robot", {})
    wait_for(lambda: not c.busy)
    c.submit("a", "drag", {"enabled": True})
    wait_for(lambda: not c.busy)
    assert c.hw.operation == "drag"
    with c.lock:
        c.expires = time.monotonic() - 1
    wait_for(lambda: c.owner is None and c.hw.operation == "idle")
    with pytest.raises(ValueError): c.heartbeat("a")


def test_drag_blocks_gripper_and_jog(controller):
    c = controller
    c.claim("a")
    c.submit("a", "connect_robot", {})
    wait_for(lambda: not c.busy)
    c.submit("a", "drag", {"enabled": True})
    wait_for(lambda: not c.busy)
    with pytest.raises(ValueError): c.submit("a", "gripper_step", {"delta": -10})
    with pytest.raises(ValueError): c.submit("a", "jog", {"joint": 1, "delta": 0.1})


def test_gripper_bounds_and_teaching_record(controller):
    c = controller
    c.claim("a")
    for action in ("connect_robot", "connect_gripper"):
        c.submit("a", action, {})
        wait_for(lambda: not c.busy)
    c.submit("a", "gripper_step", {"delta": -10})
    wait_for(lambda: not c.busy)
    assert c.hw.position == 990
    c.submit("a", "save_point", {"name": "P-test"})
    wait_for(lambda: not c.busy)
    assert c.points[0]["robot"]["flange_m_rad"]
    assert c.points[0]["gripper"]["position"] == 990
    assert c.points[0]["simulation"] is True
    assert c.points_path.exists()


def test_expired_lease_cannot_be_revived_before_stop(controller):
    c = controller
    c.claim("a")
    with c.lock:
        c.expires = time.monotonic() - 1
        with pytest.raises(ValueError): c.claim("a")
        assert c.owner is None
        assert c.stopping.is_set()


def test_sdk_lock_excludes_a_second_project_client(tmp_path):
    lib = tmp_path / "vendor/xCoreSDK-Python/Release/linux"
    lib.mkdir(parents=True)
    (lib / ("xCoreSDK_python" + importlib.machinery.EXTENSION_SUFFIXES[0])).touch()
    # Other test modules load OpenCV; the SDK lock belongs in its own process.
    code = f'''
import sys
sys.path.insert(0, {str(Path(sdk_module.__file__).parent)!r})
import sdk_runtime as module
module.__file__ = {str(tmp_path / "scripts/sdk_runtime.py")!r}
with module.sdk_runtime():
    try:
        with module.sdk_runtime():
            raise AssertionError("Second SDK client accepted")
    except RuntimeError as exc:
        assert "already held" in str(exc)
with module.sdk_runtime():
    pass
'''
    subprocess.run([sys.executable, '-c', code], check=True)


def test_failed_connection_does_not_disconnect_unconnected_sdk_object():
    class UnconnectedRobot:
        def disconnectFromRobot(self, ec):
            pytest.fail("Must not call native disconnect after a failed connection")

    hardware = Hardware()
    hardware.robot = UnconnectedRobot()
    hardware.disconnect_robot()
    assert hardware.robot is None
    assert hardware.robot_connected is False


def crash_camera_worker(frames, stop):
    os._exit(19)


def test_camera_process_failure_does_not_kill_controller(monkeypatch):
    monkeypatch.setattr(web_control, "capture_worker", crash_camera_worker)
    camera = Camera()
    try:
        camera.start()
        wait_for(lambda: camera.error is not None, timeout=5)
        assert "19" in camera.error
        assert "robot process retained" in camera.error
    finally:
        camera.stop()


def test_successful_connection_still_disconnects():
    calls = []

    class ConnectedRobot:
        def disconnectFromRobot(self, ec):
            calls.append("disconnect")
            ec["ec"] = 0

    hardware = Hardware()
    hardware.robot = ConnectedRobot()
    hardware.robot_connected = True
    hardware.disconnect_robot()
    assert calls == ["disconnect"]
    assert hardware.robot_connected is False


def test_calibration_reuses_owner_and_keeps_connection(controller, monkeypatch):
    from types import SimpleNamespace
    c = controller
    c.claim("browser")
    c.submit("browser", "connect_robot", {})
    wait_for(lambda: not c.busy)
    c.camera = SimpleNamespace(enabled=True, packet={"frame": 1})
    def sample(check):
        check()
        assert c.busy
        with pytest.raises(ValueError, match="running"):
            c.submit("browser", "jog", {"joint": 1, "delta": .1})
        return {"sample": "paired"}
    monkeypatch.setattr(c, "capture_calibration", sample)
    assert c.request_capture() == {"sample": "paired"}
    assert c.owner == "browser"
    assert c.hw.robot_state()["connected"]
    assert not c.busy


def test_calibration_failure_retains_connection(controller, monkeypatch):
    from types import SimpleNamespace
    c = controller
    c.claim("browser")
    c.submit("browser", "connect_robot", {})
    wait_for(lambda: not c.busy)
    c.camera = SimpleNamespace(enabled=True, packet={"frame": 1})
    def fail(check):
        raise RuntimeError("No fresh camera frame")
    monkeypatch.setattr(c, "capture_calibration", fail)
    with pytest.raises(RuntimeError, match="fresh"):
        c.request_capture()
    assert c.owner == "browser"
    assert c.hw.robot_state()["connected"]


def test_camera_fresh_packet_rejects_stale_and_honors_cancel():
    camera = Camera()
    camera.enabled = True
    camera.packet = {"received_monotonic": 1}
    def cancel():
        raise RuntimeError("cancelled")
    with pytest.raises(RuntimeError, match="cancelled"):
        camera.fresh_packet(2, cancel)
    camera.packet = {"received_monotonic": time.monotonic()}
    assert camera.fresh_packet(2, lambda: None) == camera.packet


def test_calibration_requires_preview_and_idle_robot(controller):
    from types import SimpleNamespace
    c = controller
    with pytest.raises(ValueError, match="preview"):
        c.request_capture()
    c.camera = SimpleNamespace(enabled=True, packet={"frame": 1})
    with pytest.raises(ValueError, match="Connect robot"):
        c.request_capture()


def test_web_save_reports_success_and_failure(controller, monkeypatch, tmp_path):
    from types import SimpleNamespace
    monkeypatch.setattr(web_control, "ROOT", tmp_path)
    saver = web_control.CalibrationSave(controller)
    saver.folder.mkdir(parents=True)
    (saver.folder / 'samples.yaml').write_text('samples:\n- split: train\n- split: validation\n')
    monkeypatch.setattr(web_control.subprocess, 'run', lambda *a, **k:
                        SimpleNamespace(returncode=0, stdout='saved', stderr=''))
    saver.run()
    assert saver.state()['train'] == saver.state()['validation'] == 1
    assert saver.state()['error'] is None
    monkeypatch.setattr(web_control.subprocess, 'run', lambda *a, **k:
                        SimpleNamespace(returncode=1, stdout='', stderr='Board not detected'))
    saver.run()
    assert saver.state()['error'] == 'Board not detected'
    assert not saver.state()['running']
