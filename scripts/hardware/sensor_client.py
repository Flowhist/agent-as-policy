"""Local task client for the existing resident web camera and gripper owners.

Standard library only: safe to import in an SDK process. Never reconnects devices
or retries commands implicitly. The web robot connection must remain disconnected.
"""

import argparse
from contextlib import contextmanager
import json
import math
import sys
import threading
import time
import urllib.error
import urllib.request
from urllib.parse import urlsplit

DEFAULT_URL = "http://127.0.0.1:8080"


class SensorClient:
    def __init__(self, url=DEFAULT_URL):
        parsed = urlsplit(url)
        if (parsed.scheme != "http" or parsed.hostname not in ("127.0.0.1", "localhost", "::1")
                or parsed.path not in ("", "/") or parsed.query or parsed.fragment or parsed.username):
            raise ValueError("Sensor service URL must be a local HTTP origin")
        self.url = url.rstrip("/")
        self.opener = urllib.request.build_opener(urllib.request.ProxyHandler({}))
        self.token = None
        self.owned = False
        self.heartbeat_error = None
        self.token = self.request("/api/session", {})["token"]
        self.state()

    def request(self, path, data=None):
        headers = {"Content-Type": "application/json"}
        if self.token:
            headers["Authorization"] = "Bearer " + self.token
        body = None if data is None else json.dumps(data, allow_nan=False).encode()
        request = urllib.request.Request(self.url+path, data=body, headers=headers)
        try:
            with self.opener.open(request, timeout=6) as response:
                return json.load(response)
        except urllib.error.HTTPError as exc:
            raise RuntimeError(f"Resident service rejected request: {exc.read().decode()}") from exc
        except (OSError, ValueError) as exc:
            raise RuntimeError("Resident service unavailable or response unconfirmed; no command retry or direct-device fallback") from exc

    def state(self):
        state = self.request("/api/state")
        if state["simulation"]:
            raise RuntimeError("Simulation cannot provide real task sensor data")
        return state

    def capture(self, *, after=0.):
        packet = self.request("/api/camera/capture", {"after": after})
        self.check_age(packet["received_monotonic"])
        return packet

    def gripper_snapshot(self, **conditions):
        packet = self.request("/api/gripper/snapshot", conditions)
        self.check_age(packet["state"]["sampled_monotonic"])
        return packet

    def verify_holding(self, position):
        """Require a fresh five-sample contact window relative to the grasp result."""
        if type(position) is not int or not 0 <= position <= 1000:
            raise ValueError("Holding position must be a measured gripper register value")
        return self.gripper_snapshot(after=time.monotonic(), stable_samples=5,
                                     expected_status=2, expected_position=position,
                                     position_tolerance=3)

    @staticmethod
    def check_age(sampled):
        age = time.monotonic()-sampled
        if not math.isfinite(age) or not 0 <= age <= .5:
            raise RuntimeError("Sensor response is stale or belongs to a different host clock")

    def check_control(self):
        if not self.owned or self.heartbeat_error:
            raise RuntimeError(self.heartbeat_error or "Acquire task control first")

    @contextmanager
    def control(self):
        if self.owned:
            raise RuntimeError("Task control context is already active")
        if self.state()["robot"].get("connected"):
            raise RuntimeError("Disconnect the web robot before using the task sensor client")
        self.request("/api/claim", {})
        self.owned = True
        self.heartbeat_error = None
        stop = threading.Event()

        def heartbeat():
            while not stop.wait(1):
                try:
                    self.request("/api/heartbeat", {})
                except Exception as exc:
                    self.heartbeat_error = str(exc)
                    return

        thread = threading.Thread(target=heartbeat, daemon=True)
        thread.start()
        try:
            if self.state()["robot"].get("connected"):
                raise RuntimeError("Web robot connection changed during control acquisition")
            yield self
            self.check_control()
        finally:
            unwinding = sys.exc_info()[0] is not None
            stop.set()
            thread.join(timeout=7)
            self.owned = False
            try:
                self.request("/api/release", {})
                self.wait_released()
            except RuntimeError:
                if not unwinding:
                    raise

    def wait_released(self, timeout=8.):
        """Release requests an asynchronous stop; finish that handoff before returning."""
        deadline = time.monotonic()+timeout
        while True:
            state = self.state()
            if not state["stopping"] and not state["busy"] and not state["owner"]:
                return
            if time.monotonic() >= deadline:
                raise RuntimeError("Control release did not finish; check service state, do not retry the action")
            time.sleep(.05)

    def action(self, action, *, on_feedback=None, **args):
        if action not in {"connect_gripper", "disconnect_gripper", "gripper_step", "gripper_target", "grasp_close"}:
            raise ValueError("Task client only sends guarded gripper actions")
        self.check_control()
        request_id = self.request("/api/action", {"action": action, "args": args})["request_id"]
        deadline = time.monotonic()+25
        sequence = None
        while time.monotonic() < deadline:
            self.check_control()
            state = self.state()
            gripper = state["gripper"]
            if on_feedback and gripper.get("sequence") != sequence:
                sequence = gripper.get("sequence")
                on_feedback(gripper)
            for event in state["events"]:
                if event.get("request_id") == request_id:
                    if event["event"] == "completed":
                        return event.get("result")
                    if event["event"] in ("error", "cancelled"):
                        raise RuntimeError(f"Gripper action {request_id}: {event}")
            if not state["owner"] or state["stopping"]:
                raise RuntimeError(f"Control lost during action {request_id}; outcome must be checked, do not retry")
            time.sleep(.1)
        raise RuntimeError(f"Unconfirmed action {request_id}; do not retry")

    def camera_enabled(self, enabled):
        self.check_control()
        self.request("/api/camera", {"enabled": enabled})
        if enabled:
            deadline = time.monotonic()+7
            while time.monotonic() < deadline:
                self.check_control()
                camera = self.state()["camera"]
                if camera["error"]:
                    raise RuntimeError(camera["error"])
                if camera["enabled"] and 0 <= time.time()-camera["updated"] <= .5:
                    return
                time.sleep(.1)
            raise RuntimeError("Camera did not finish warmup; no automatic restart")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("operation", choices=("status", "connect", "disconnect"))
    parser.add_argument("--camera", action="store_true")
    parser.add_argument("--gripper", action="store_true")
    parser.add_argument("--web-url", default=DEFAULT_URL)
    args = parser.parse_args()
    if args.operation == 'status':
        if args.camera or args.gripper:
            parser.error('status does not take connection flags')
        try:
            state = SensorClient(args.web_url).state()
            keys = ('robot', 'gripper', 'camera', 'owner', 'occupied', 'busy', 'stopping')
            print(json.dumps({key: state[key] for key in keys}, ensure_ascii=False))
            return 0
        except (RuntimeError, ValueError) as exc:
            print(f'Status query failed: {exc}', file=sys.stderr)
            return 1
    if not (args.camera or args.gripper):
        parser.error("Select --camera and/or --gripper explicitly")
    try:
        with SensorClient(args.web_url).control() as client:
            if args.gripper:
                client.action(args.operation+"_gripper")
            if args.camera:
                client.camera_enabled(args.operation == "connect")
        print("Resident sensor connections updated; no motion, force or homing commands sent")
        return 0
    except (RuntimeError, ValueError) as exc:
        print(f"Sensor connection rejected: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
