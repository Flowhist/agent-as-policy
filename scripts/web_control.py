"""LAN teaching console. Start with python3 scripts/web_control.py."""

import argparse
import base64
from collections import deque
from datetime import datetime, timezone
import fcntl
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import math
import multiprocessing
import os
from pathlib import Path
import queue
import secrets
import signal
import subprocess
import sys
import threading
import time
from urllib.parse import urlsplit

if __package__ in (None, ''):
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from scripts.hardware.web_hardware import Hardware, SimHardware
from scripts.hardware.gripper_monitor import GripperMonitor
from scripts.hardware.control_settings import load_settings, read_settings, save_settings

ROOT = Path(__file__).resolve().parent.parent
WEB = ROOT / "web"
LEASE_SECONDS = 5


def utc():
    return datetime.now(timezone.utc).isoformat()


def validate(action, data):
    if action in {'joint_target', 'jog'} and 'rate' not in data:
        data = {**data, 'rate': load_settings()['robot']['sdk_rate']}
    if action == "joint_target":
        joint, target, rate = data.get("joint"), data.get("target_deg"), data.get("rate", 0.01)
        if type(joint) is not int or joint not in range(1, 7):
            raise ValueError("Joint must be 1..6")
        if any(type(v) not in (int, float) or not math.isfinite(v) for v in (target, rate)):
            raise ValueError("Finite target and rate required")
        if not -360 <= target <= 360 or not 0.01 <= rate <= 1.0:
            raise ValueError("Invalid target or rate")
        return {"joint": joint, "target_deg": target, "rate": rate}
    if action == "gripper_target":
        target = data.get("target")
        if type(target) is not int or not 0 <= target <= 1000:
            raise ValueError("Gripper target must be 0..1000")
        return {"target": target}
    if action == "grasp_close":
        target = data.get("target_position", 0)
        if type(target) is not int or not 0 <= target < 1000:
            raise ValueError("Closing target must be 0..999")
        return {"target_position": target}
    if action == "jog":
        joint, delta, rate = data.get("joint"), data.get("delta"), data.get("rate", 0.01)
        if type(joint) is not int or joint not in range(1, 7):
            raise ValueError("Joint must be 1..6")
        for value in (delta, rate):
            if type(value) not in (int, float) or not math.isfinite(value):
                raise ValueError("Finite numeric delta and rate required")
        if not 0.01 <= abs(delta) <= 50:
            raise ValueError("Joint step must be 0.01..50 degrees")
        if not 0.01 <= rate <= 1.0:
            raise ValueError("Rate must be 0.01..1.0")
        return {"joint": joint, "delta": delta, "rate": rate}
    if action == "gripper_step":
        delta = data.get("delta")
        if type(delta) is not int or not 1 <= abs(delta) <= 500:
            raise ValueError("Gripper step must be 1..500 raw units (0.1..50%)")
        return {"delta": delta}
    if action == "drag":
        if type(data.get("enabled")) is not bool:
            raise ValueError("Boolean enabled required")
        return {"enabled": data["enabled"]}
    if action == "save_point":
        name = data.get("name", "")
        if not isinstance(name, str) or len(name) > 80:
            raise ValueError("Point name must be at most 80 characters")
        return {"name": name.strip()}
    if action in {"capture_calibration", "connect_robot", "disconnect_robot", "connect_gripper", "disconnect_gripper"}:
        return {}
    raise ValueError("Unknown action")


def capture_worker(frames, stop):
    """Keep RealSense/OpenCV native libraries out of the robot SDK process."""
    pipeline = None
    started = False
    frames.cancel_join_thread()
    try:
        import cv2
        import numpy as np
        import pyrealsense2 as rs
        pipeline = rs.pipeline()
        config = rs.config()
        config.enable_device("141322252639")
        config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 10)
        profile = pipeline.start(config)
        started = True
        intr = profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
        camera_info = {"serial": "141322252639", "width": intr.width, "height": intr.height,
                       "fps": 10, "fx": intr.fx, "fy": intr.fy, "ppx": intr.ppx, "ppy": intr.ppy,
                       "distortion_model": str(intr.model), "coefficients": list(intr.coeffs),
                       "note": "Raw RGB; do not treat inverse_brown_conrady as OpenCV Brown distortion"}
        from scripts.vision.camera_settings import apply_rgb_settings
        apply_rgb_settings(profile, rs)
        warmup = 0
        previous_number = previous_timestamp = None
        while not stop.is_set():
            result = pipeline.wait_for_frames(2000)
            received = time.monotonic()
            frame = result.get_color_frame()
            warmup += 1
            if not frame:
                continue
            number, timestamp = frame.get_frame_number(), frame.get_timestamp()
            if previous_number is not None:
                if number < previous_number or timestamp < previous_timestamp:
                    raise RuntimeError("Camera frame clock reset; restart preview explicitly")
                if number == previous_number or timestamp == previous_timestamp:
                    continue
            previous_number, previous_timestamp = number, timestamp
            if warmup < 30:
                continue
            ok, jpg = cv2.imencode(".jpg", np.asanyarray(frame.get_data()),
                                  [cv2.IMWRITE_JPEG_QUALITY, 85])
            if ok:
                png_ok, png = cv2.imencode(".png", np.asanyarray(frame.get_data()))
                if not png_ok:
                    raise RuntimeError("PNG encoding failed")
                packet = {"png_base64": base64.b64encode(png.tobytes()).decode(),
                          "received_monotonic": received, "camera": camera_info,
                          "frame": {"timestamp_ms": frame.get_timestamp(),
                                    "clock_domain": str(frame.get_frame_timestamp_domain()),
                                    "frame_number": frame.get_frame_number(), "host_received_utc": utc()}}
                try:
                    frames.put_nowait((jpg.tobytes(), time.time(), None, packet))
                except queue.Full:
                    pass
    except Exception as exc:
        try:
            frames.put((None, 0, str(exc)), timeout=0.5)
        except queue.Full:
            pass
    finally:
        if started:
            pipeline.stop()


class Camera:
    def __init__(self, simulate=False):
        self.simulate = simulate
        self.frame = None
        self.updated = 0
        self.error = None
        self.enabled = False
        self.thread = None
        self.lock = threading.Lock()
        self.packet = None

    def start(self):
        with self.lock:
            if self.thread and self.thread.is_alive():
                return
            self.enabled = True
            self.error = None
            self.frame = None
            self.updated = 0
            self.packet = None
            self.thread = threading.Thread(target=self.run, daemon=True)
            self.thread.start()

    def run(self):
        process = None
        frames = None
        stop = None
        try:
            if self.simulate:
                files = sorted((ROOT / "captures").glob("*.jpg"))
                if not files:
                    raise RuntimeError("No simulation photo available")
                self.frame = files[-1].read_bytes()
                while self.enabled:
                    self.updated = time.time()
                    time.sleep(0.1)
                return
            context = multiprocessing.get_context("spawn")
            frames = context.Queue(maxsize=1)
            stop = context.Event()
            process = context.Process(target=capture_worker, args=(frames, stop), daemon=True)
            process.start()
            while self.enabled:
                try:
                    item = frames.get(timeout=0.2)
                    image, timestamp, error = item[:3]
                    if error:
                        raise RuntimeError(error)
                    self.frame, self.updated = image, timestamp
                    self.packet = item[3] if len(item) > 3 else None
                except queue.Empty:
                    if not process.is_alive():
                        raise RuntimeError(f"Camera process exited ({process.exitcode}); robot process retained")
        except Exception as exc:
            self.error = str(exc)
        finally:
            if stop is not None:
                stop.set()
            if process is not None and process.pid is not None:
                process.join(timeout=2.5)
                if process.is_alive():
                    process.terminate()
                    process.join(timeout=0.5)
            if frames is not None:
                frames.close()
            self.enabled = False

    def stop(self):
        self.enabled = False
        if self.thread:
            self.thread.join(timeout=3.5)

    def fresh_packet(self, after, check):
        deadline = time.monotonic() + 3
        while time.monotonic() < deadline:
            check()
            if not self.enabled or self.error:
                raise RuntimeError(self.error or "Enable web camera preview first")
            packet = self.packet
            if (packet and packet["received_monotonic"] > after
                    and 0 <= time.monotonic()-packet["received_monotonic"] <= .5):
                return packet
            time.sleep(0.02)
        raise RuntimeError("No fresh camera frame; sample rejected")


class Controller:
    def __init__(self, hardware, data_dir, simulate=False):
        self.hw = hardware
        self.simulate = simulate
        self.data_dir = Path(data_dir)
        self.data_dir.mkdir(parents=True, exist_ok=True)
        self.log = (self.data_dir / ("web-" + datetime.now().strftime("%Y%m%d-%H%M%S") + ".jsonl")).open("a")
        self.lock = threading.RLock()
        self.owner = None
        self.expires = 0
        self.pending = None
        self.busy = False
        self.stopping = threading.Event()
        self.shutdown = threading.Event()
        self.events = deque(maxlen=30)
        self.robot = {"connected": False}
        self.gripper = {"connected": False}
        self.updated = 0
        self.error = None
        self.camera = None
        self.capture_done = threading.Event()
        self.capture_result = None
        self.capture_request_lock = threading.Lock()
        self.points_path = self.data_dir / "teaching-points.jsonl"
        self.hw.on_robot = lambda state: self.feedback("robot", state)
        self.points = []
        if self.points_path.exists():
            for line in self.points_path.read_text().splitlines():
                try:
                    self.points.append(json.loads(line))
                except ValueError:
                    pass
        self.gripper_monitor = GripperMonitor(
            self.hw, lambda state: self.feedback("gripper", state), self.gripper_failed)
        self.thread = threading.Thread(target=self.run, daemon=True)
        self.thread.start()

    def feedback(self, kind, state):
        with self.lock:
            setattr(self, kind, state)
            self.updated = time.time()

    def gripper_failed(self, message):
        self.error = message
        self.feedback("gripper", {"connected": False, "error": message})
        self.emit("gripper_feedback_failed", message=message)
        self.stop("gripper_feedback_failed")

    def disconnect_gripper(self):
        return self.gripper_monitor.call(self.hw.disconnect_gripper)

    def emit(self, event, **fields):
        with self.lock:
            record = {"time": utc(), "event": event, **fields}
            self.events.appendleft(record)
            self.log.write(json.dumps(record, ensure_ascii=False) + "\n")
            self.log.flush()

    def claim(self, token):
        with self.lock:
            if self.owner and time.monotonic() >= self.expires:
                self.stop("heartbeat_expired")
            if self.stopping.is_set() or self.busy:
                raise ValueError("Wait for the current action or stop to finish")
            if self.owner and self.owner != token:
                raise ValueError("Another browser owns control")
            self.owner = token
            self.expires = time.monotonic() + LEASE_SECONDS
            self.emit("control_acquired")

    def heartbeat(self, token):
        with self.lock:
            if self.owner != token or time.monotonic() >= self.expires:
                raise ValueError("Control lease expired; reacquire control")
            self.expires = time.monotonic() + LEASE_SECONDS

    def stop(self, reason="operator_stop"):
        with self.lock:
            self.owner = None
            if self.pending:
                self.emit("cancelled", request_id=self.pending[3], reason=reason)
            self.pending = None
            self.stopping.set()
            self.emit("stop_requested", reason=reason,
                      note="Gripper has no independent stop command")

    def check(self, token):
        with self.lock:
            if (self.shutdown.is_set() or self.stopping.is_set() or self.owner != token
                    or time.monotonic() >= self.expires):
                raise RuntimeError("Motion cancelled or control heartbeat lost")

    def submit(self, token, action, data):
        with self.lock:
            self.check(token)
            args = validate(action, data)
            if action in {'gripper_target', 'gripper_step', 'grasp_close'}:
                args['force_percent'] = load_settings()['gripper']['force_percent']
            if self.busy or self.pending:
                raise ValueError("An action is running; commands are not queued")
            if self.robot.get("operation") == "drag" and action not in {"drag", "save_point", "disconnect_robot"}:
                raise ValueError("Exit drag teaching before other commands")
            self.busy = True
            request_id = secrets.token_hex(16)
            self.pending = (token, action, args, request_id)
            self.emit("accepted", action=action, args=args, request_id=request_id)
            return request_id

    def poll(self):
        robot = self.hw.robot_state()
        with self.lock:
            self.robot = robot
            self.updated = time.time()

    def save_point(self, name, check):
        a = self.hw.robot_state()
        if not a["connected"] or a["operation"] not in ("idle", "drag"):
            raise RuntimeError("Stop moving before recording a teaching point")
        samples = [a["joints_deg"]]
        for _ in range(4):
            time.sleep(0.1)
            check()
            b = self.hw.robot_state()
            samples.append(b["joints_deg"])
        if any(max(axis) - min(axis) > 0.01 for axis in zip(*samples)):
            raise RuntimeError("Robot is not stationary; point not saved")
        gripper = (self.gripper_monitor.snapshot()["state"] if self.gripper.get("connected")
                   else {"connected": False})
        point = {"id": secrets.token_hex(6), "name": name or f"P{len(self.points) + 1:03d}",
                 "time": utc(), "robot": b, "gripper": gripper,
                 "simulation": self.simulate,
                 "note": "Teaching point only; no automatic replay or synchronized calibration image"}
        with self.points_path.open("a") as f:
            f.write(json.dumps(point, ensure_ascii=False) + "\n")
        with self.lock:
            self.points.append(point)
        return {"point": point["name"]}

    def run_action(self, token, action, args):
        check = lambda: self.check(token)
        check()
        if action == "capture_calibration":
            return self.capture_calibration(check)
        if action in {"connect_gripper", "disconnect_gripper", "gripper_target", "gripper_step", "grasp_close"}:
            def operate():
                check()
                if self.robot.get("operation") == "drag":
                    raise RuntimeError("Exit drag before operating gripper")
                if action == "gripper_target":
                    return self.hw.gripper_step(**args, delta=None, check=check)
                if action == "gripper_step":
                    return self.hw.gripper_step(**args, check=check)
                if action == "grasp_close":
                    if self.simulate:
                        raise RuntimeError("Simulation cannot confirm real gripper contact")
                    from scripts.hardware.grasp_close import close_until_contact
                    return close_until_contact(self.hw, self.emit, check=check, **args)
                return getattr(self.hw, action)()
            return self.gripper_monitor.call(operate)
        if action == "jog":
            return self.hw.jog(**args, check=check)
        if action == "joint_target":
            return self.hw.jog(**args, delta=None, check=check)
        if action == "drag":
            return self.hw.drag(**args)
        if action == "save_point":
            return self.save_point(**args, check=check)
        if action == "disconnect_robot":
            self.hw.safe_stop()
        return getattr(self.hw, action)()

    def capture_calibration(self, check):
        from scripts.calibration.calibration_capture import stable
        if self.simulate:
            raise RuntimeError("Simulation cannot create real calibration samples")
        def read():
            check()
            start = utc()
            state = self.hw.robot_state()
            if (not state["connected"] or state["mode"] != "manual"
                    or state["operation"] != "idle" or state["power"] not in ("on", "off")):
                raise RuntimeError("Exit drag and stop moving before sampling; keep robot connected")
            return {"read_start_utc": start, "read_end_utc": utc(),
                    "joints_rad": [math.radians(v) for v in state["joints_deg"]],
                    "flange_in_base_m_rad": state["flange_m_rad"],
                    "T_base_flange_row_major": list(self.hw.sdk.utility.postureToTransArray(state["flange_m_rad"])),
                    "mode": state["mode"], "operation": state["operation"], "power": state["power"]}
        readings = [read()]
        time.sleep(0.3)
        readings.append(read())
        stable(readings)
        # Leave several frame periods after the pre-read, excluding camera buffers.
        packet = self.camera.fresh_packet(time.monotonic() + 0.4, check)
        readings.append(read())
        time.sleep(0.2)
        readings.append(read())
        return {**packet, "robot_readings": readings, "stability": stable(readings),
                "robot": {"source": "web_single_owner", "sdk": self.hw.sdk.BaseRobot.sdkVersion()}}

    def request_capture(self):
        if not self.capture_request_lock.acquire(blocking=False):
            raise ValueError("Another calibration request is running")
        try:
            return self._request_capture()
        finally:
            self.capture_request_lock.release()

    def _request_capture(self):
        with self.lock:
            if self.busy or self.pending:
                raise ValueError("An action is running")
            if not self.camera or not self.camera.enabled or not self.camera.packet:
                raise ValueError("Enable camera preview and wait for a live frame")
            if not self.robot.get("connected") or self.robot.get("operation") != "idle":
                raise ValueError("Connect robot and exit drag before sampling")
            self.capture_result = None
            self.capture_done.clear()
            self.submit(self.owner, "capture_calibration", {})
        if not self.capture_done.wait(12):
            raise RuntimeError("Capture timed out; do not retry until controller is idle")
        result = self.capture_result
        if not result or "error" in result:
            raise RuntimeError((result or {}).get("error", "Capture cancelled"))
        return result

    def run(self):
        next_poll = 0
        while not self.shutdown.is_set():
            job = None
            try:
                with self.lock:
                    if self.owner and time.monotonic() >= self.expires:
                        self.stop("heartbeat_expired")
                    job = self.pending
                    self.pending = None
                if self.stopping.is_set():
                    if job:
                        self.emit("cancelled", request_id=job[3], reason="stop_requested")
                    try:
                        self.hw.safe_stop()
                        self.emit("stopped")
                    finally:
                        self.stopping.clear()
                        self.busy = False
                elif job:
                    token, action, args, request_id = job
                    if action == "capture_calibration":
                        try:
                            self.capture_result = self.run_action(token, action, args)
                            self.emit("completed", action=action)
                        except Exception as exc:
                            self.capture_result = {"error": str(exc)}
                            self.emit("capture_rejected", message=str(exc))
                        finally:
                            self.busy = False
                            self.capture_done.set()
                        continue
                    result = self.run_action(token, action, args)
                    self.emit("completed", action=action, result=result, request_id=request_id)
                    self.error = None
                    self.busy = False
                if job or time.monotonic() >= next_poll:
                    self.poll()
                    next_poll = time.monotonic() + 0.25
            except Exception as exc:
                self.error = str(exc)
                self.emit("error", message=str(exc), request_id=job[3] if job else None)
                with self.lock:
                    self.owner = None
                    self.pending = None
                    self.busy = True
                try:
                    self.hw.safe_stop()
                except Exception as cleanup:
                    self.emit("stop_failed", message=str(cleanup))
                # Stop polling failed connections; recovery is explicit.
                for method in (self.hw.disconnect_robot, self.disconnect_gripper):
                    try:
                        method()
                    except Exception as cleanup:
                        self.emit("disconnect_failed", message=str(cleanup))
                self.robot = {"connected": False}
                self.gripper = {"connected": False}
                self.stopping.clear()
                self.busy = False
            self.shutdown.wait(0.05)
        try:
            self.hw.safe_stop()
        except Exception as exc:
            self.emit("shutdown_stop_failed", message=str(exc))
        finally:
            for method in (self.hw.disconnect_robot, self.disconnect_gripper):
                try:
                    method()
                except Exception as exc:
                    self.emit("shutdown_disconnect_failed", message=str(exc))

    def state(self, token):
        with self.lock:
            return {"simulation": self.simulate, "robot": self.robot, "gripper": self.gripper,
                    "settings": read_settings(),
                    "owner": self.owner == token, "occupied": self.owner is not None,
                    "busy": self.busy, "stopping": self.stopping.is_set(),
                    "updated": self.updated, "error": self.error,
                    "events": list(self.events), "points": self.points[-50:]}

    def close(self):
        self.shutdown.set()
        self.stopping.set()
        self.gripper_monitor.close()
        self.thread.join(timeout=10)
        if not self.thread.is_alive():
            self.log.close()


class Handler(BaseHTTPRequestHandler):
    def log_message(self, *args):
        pass

    def respond(self, code, payload, content_type="application/json; charset=utf-8"):
        body = json.dumps(payload, ensure_ascii=False).encode() if isinstance(payload, (dict, list)) else payload
        self.send_response(code)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.send_header("X-Content-Type-Options", "nosniff")
        self.send_header("X-Frame-Options", "DENY")
        self.end_headers()
        try:
            self.wfile.write(body)
        except (BrokenPipeError, ConnectionResetError):
            pass

    def token(self):
        token = self.headers.get("Authorization", "").removeprefix("Bearer ")
        with self.server.auth_lock:
            if token not in self.server.sessions:
                raise PermissionError("Browser session expired")
        return token

    def do_GET(self):
        path = urlsplit(self.path).path
        static = {"/": ("index.html", "text/html; charset=utf-8"),
                  "/app.js": ("app.js", "text/javascript; charset=utf-8"),
                  "/style.css": ("style.css", "text/css; charset=utf-8"),
                  "/lucide.min.js": ("lucide.min.js", "text/javascript; charset=utf-8")}
        try:
            if path in static:
                name, mime = static[path]
                return self.respond(200, (WEB / name).read_bytes(), mime)
            token = self.token()
            if path == "/api/state":
                result = self.server.control.state(token)
                camera = self.server.camera
                result["camera"] = {"enabled": camera.enabled, "updated": camera.updated, "error": camera.error}
                result["calibration"] = self.server.calibration.state()
                return self.respond(200, result)
            if path == "/api/camera.jpg":
                camera = self.server.camera
                if not camera.enabled or not camera.frame or time.time() - camera.updated > 2:
                    return self.respond(503, {"error": camera.error or "Camera warming up or offline"})
                return self.respond(200, camera.frame, "image/jpeg")
            if path == "/api/points":
                with self.server.control.lock:
                    return self.respond(200, list(self.server.control.points))
            self.respond(404, {"error": "Not found"})
        except PermissionError as exc:
            self.respond(401, {"error": str(exc)})
        except Exception as exc:
            self.respond(500, {"error": str(exc)})

    def do_POST(self):
        try:
            # Exact Origin/Host matching blocks cross-site browser requests.
            if self.headers.get("Origin") not in (None, "http://" + self.headers.get("Host", "")):
                raise PermissionError("Cross-origin request rejected")
            if self.headers.get("Content-Type", "").split(";")[0] != "application/json":
                raise ValueError("JSON body required")
            length = int(self.headers.get("Content-Length", "0"))
            if not 0 < length <= 4096:
                raise ValueError("Invalid request size")
            data = json.loads(self.rfile.read(length))
            if not isinstance(data, dict):
                raise ValueError("JSON object required")
            path = urlsplit(self.path).path
            if path == "/api/session":
                with self.server.auth_lock:
                    token = secrets.token_urlsafe(32)
                    self.server.sessions.add(token)
                return self.respond(200, {"token": token})
            token = self.token()
            control = self.server.control
            if path in {"/api/camera/capture", "/api/gripper/snapshot"}:
                if self.client_address[0] not in ("127.0.0.1", "::1"):
                    raise PermissionError("Task sensor clients must run on the robot host")
                if control.simulate:
                    raise ValueError("Simulation cannot provide real task sensor data")
                if path == "/api/gripper/snapshot":
                    return self.respond(200, control.gripper_monitor.snapshot(**data))
                after = data.get("after", 0.)
                if type(after) not in (int, float) or not math.isfinite(after) or not 0 <= after <= time.monotonic():
                    raise ValueError("after must be a past host monotonic timestamp")
                # Span several frame periods after the request to exclude queued frames.
                packet = self.server.camera.fresh_packet(max(after, time.monotonic())+.4, lambda: None)
                return self.respond(200, packet)
            if path == "/api/calibration/save":
                control.check(token)
                self.server.calibration.start()
                return self.respond(200, {"ok": True})
            if path == "/api/calibration/capture":
                if self.client_address[0] not in ("127.0.0.1", "::1"):
                    raise PermissionError("Calibration script must run on the robot host")
                return self.respond(200, control.request_capture())
            if path == "/api/claim":
                control.claim(token)
            elif path == "/api/heartbeat":
                control.heartbeat(token)
            elif path == "/api/stop":
                control.stop()
            elif path == "/api/release":
                with control.lock:
                    if control.owner == token:
                        control.stop("control_released")
            elif path == "/api/settings":
                with control.lock:
                    control.check(token)
                    if control.busy or control.pending or control.robot.get('operation') not in (None, 'idle'):
                        raise ValueError('Settings require idle devices')
                    if control.gripper.get('connected') and control.gripper.get('status') == 0:
                        raise ValueError('Wait for the gripper to stop')
                    result = save_settings(data.get('values'), data.get('revision'))
                    control.emit('settings_saved', **result)
                return self.respond(200, result)
            elif path == "/api/action":
                request_id = control.submit(token, data.get("action"), data.get("args", {}))
                return self.respond(200, {"ok": True, "request_id": request_id})
            elif path == "/api/camera":
                control.check(token)
                if type(data.get("enabled")) is not bool:
                    raise ValueError("Boolean enabled required")
                if data["enabled"]:
                    self.server.camera.start()
                else:
                    self.server.camera.stop()
            else:
                return self.respond(404, {"error": "Not found"})
            self.respond(200, {"ok": True})
        except PermissionError as exc:
            self.respond(401, {"error": str(exc)})
        except (ValueError, TypeError, AttributeError, RuntimeError) as exc:
            self.respond(409, {"error": str(exc)})


class CalibrationSave:
    """Run detection and YAML persistence outside the native robot SDK process."""

    def __init__(self, control):
        self.control = control
        self.lock = threading.Lock()
        self.running = False
        self.message = ""
        self.error = None
        self.train = 0
        self.validation = 0
        self.folder = ROOT / "captures/calibration/session-01"
        self.refresh_counts()

    def refresh_counts(self):
        import yaml
        path = self.folder / "samples.yaml"
        if path.exists():
            data = yaml.safe_load(path.read_text())
            self.train = sum(s["split"] == "train" for s in data["samples"])
            self.validation = sum(s["split"] == "validation" for s in data["samples"])

    def state(self):
        with self.lock:
            return {"running": self.running, "message": self.message, "error": self.error,
                    "train": self.train, "validation": self.validation, "session": "session-01"}

    def start(self):
        with self.lock:
            if self.running:
                raise ValueError("Calibration sample is already being saved")
            c = self.control
            with c.lock:
                if c.simulate:
                    raise ValueError("Simulation cannot save real calibration samples")
                if c.busy or c.stopping.is_set() or not c.robot.get("connected") or c.robot.get("operation") != "idle":
                    raise ValueError("Keep robot connected and exit drag before sampling")
                if not c.camera or not c.camera.enabled or not c.camera.packet:
                    raise ValueError("Enable camera preview first")
            self.running = True
            self.error = None
            self.message = ""
            threading.Thread(target=self.run, daemon=True).start()

    def run(self):
        try:
            result = subprocess.run([sys.executable, "-m", "scripts.calibration.calibration_capture"],
                                    cwd=ROOT, capture_output=True, text=True, timeout=30)
            if result.returncode:
                raise RuntimeError((result.stderr or result.stdout).strip())
            with self.lock:
                self.refresh_counts()
                self.message = "采样已保存"
            self.control.emit("calibration_saved", train=self.train, validation=self.validation)
        except Exception as exc:
            with self.lock:
                self.error = str(exc)
            self.control.emit("capture_rejected", message=str(exc))
        finally:
            with self.lock:
                self.running = False


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=8080)
    parser.add_argument("--simulate", action="store_true")
    args = parser.parse_args()
    lock_path = ROOT / "logs" / ("web-simulation.lock" if args.simulate else "web-control.lock")
    lock_path.parent.mkdir(parents=True, exist_ok=True)
    instance_lock = lock_path.open("w")
    fcntl.flock(instance_lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    sdk_logs = ROOT / "logs/sdk"
    sdk_logs.mkdir(parents=True, exist_ok=True)
    os.chdir(sdk_logs)
    server = ThreadingHTTPServer((args.host, args.port), Handler)
    server.daemon_threads = True
    server.auth_lock = threading.Lock()
    server.sessions = set()
    server.control = Controller(SimHardware() if args.simulate else Hardware(),
                                ROOT / "logs" / ("web-simulation" if args.simulate else "web"), args.simulate)
    server.camera = Camera(args.simulate)
    server.control.camera = server.camera
    server.calibration = CalibrationSave(server.control)

    def stop_server(*_):
        server.control.stop("server_shutdown")
        threading.Thread(target=server.shutdown, daemon=True).start()

    signal.signal(signal.SIGTERM, stop_server)
    signal.signal(signal.SIGINT, stop_server)
    print(f"Web console: http://{args.host}:{args.port} (no access code)", flush=True)
    print(f"Simulation: {args.simulate}. Hardware is disconnected until requested in the browser.", flush=True)
    try:
        server.serve_forever(poll_interval=0.2)
    finally:
        server.control.close()
        server.camera.stop()
        server.server_close()
        instance_lock.close()


if __name__ == "__main__":
    main()
