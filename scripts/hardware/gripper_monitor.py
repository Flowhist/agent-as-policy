"""Persistent serial sampling shared by the web controller and task clients."""

from collections import deque
from datetime import datetime, timezone
import math
import threading
import time


class GripperMonitor:
    def __init__(self, hardware, publish, failed):
        self.hardware = hardware
        self.publish = publish
        self.failed = failed
        self.bus = threading.Lock()
        self.condition = threading.Condition()
        self.samples = deque(maxlen=32)
        self.error = None
        self.connected = False
        self.busy = False
        self.sequence = 0
        self.closed = threading.Event()
        hardware.on_gripper = self.record
        self.thread = threading.Thread(target=self.run, daemon=True)
        self.thread.start()

    def record(self, state):
        with self.condition:
            self.sequence += 1
            self.connected = bool(state.get("connected"))
            sample = dict(state, sequence=self.sequence, sampled_monotonic=time.monotonic(),
                          sampled_utc=datetime.now(timezone.utc).isoformat())
            if state.get("connected"):
                self.samples.append(sample)
            else:
                self.samples.clear()
            self.condition.notify_all()
        self.publish(sample)

    def sample(self):
        sequence = self.sequence
        state = self.hardware.gripper_state()
        if self.sequence == sequence:
            self.record(state)

    def call(self, operation):
        # A command owns the serial bus; its existing feedback loop feeds record().
        # Clear the old stability window before any state-changing operation.
        with self.bus:
            with self.condition:
                self.busy = True
                self.samples.clear()
                self.condition.notify_all()
            try:
                result = operation()
                self.sample()
                with self.condition:
                    self.error = None
                return result
            except Exception as exc:
                with self.condition:
                    self.error = str(exc)
                    self.samples.clear()
                raise
            finally:
                with self.condition:
                    self.busy = False
                    self.condition.notify_all()

    def run(self):
        while not self.closed.is_set():
            failure = None
            if self.bus.acquire(blocking=False):
                try:
                    if self.error is None:
                        self.sample()
                except Exception as exc:
                    failure = str(exc)
                    try:
                        self.hardware.disconnect_gripper()
                    except Exception as cleanup:
                        failure += f"; disconnect failed: {cleanup}"
                    with self.condition:
                        self.error = failure
                        self.connected = False
                        self.samples.clear()
                        self.condition.notify_all()
                finally:
                    self.bus.release()
            if failure is not None:
                self.failed(failure)
            self.closed.wait(.15)

    def snapshot(self, *, after=0., stable_samples=1, expected_status=None,
                 expected_position=None, position_tolerance=3):
        if type(after) not in (int, float) or not math.isfinite(after) or not 0 <= after <= time.monotonic():
            raise ValueError("after must be a past host monotonic timestamp")
        if type(stable_samples) is not int or not 1 <= stable_samples <= 5:
            raise ValueError("stable_samples must be 1..5")
        if expected_status is not None and (type(expected_status) is not int or expected_status not in (1, 2)):
            raise ValueError("expected_status must be 1 (empty) or 2 (contact)")
        if expected_position is not None and (type(expected_position) is not int or not 0 <= expected_position <= 1000):
            raise ValueError("expected_position must be 0..1000")
        if type(position_tolerance) is not int or not 0 <= position_tolerance <= 3:
            raise ValueError("position_tolerance must be 0..3")
        deadline = time.monotonic() + 3
        with self.condition:
            while True:
                now = time.monotonic()
                if self.closed.is_set() or self.error:
                    raise RuntimeError(self.error or "Gripper monitor stopped")
                if not self.connected and not self.busy:
                    raise RuntimeError("Connect the resident gripper first")
                rows = list(self.samples)[-stable_samples:]
                if not self.busy and rows:
                    latest = rows[-1]
                    fresh = 0 <= now-latest["sampled_monotonic"] <= .5
                    eligible = fresh and latest["sampled_monotonic"] > after
                    if eligible and expected_status is not None and latest["status"] != expected_status:
                        raise RuntimeError("Gripper status differs from expected state")
                    if eligible and expected_status is not None and latest["initialized"] != 1:
                        raise RuntimeError("Gripper is not initialized")
                    if eligible and expected_position is not None and abs(latest["position"]-expected_position) > position_tolerance:
                        raise RuntimeError("Gripper position differs from expected state")
                    if (fresh and len(rows) == stable_samples
                            and rows[0]["sampled_monotonic"] > after
                            and now-rows[0]["sampled_monotonic"] <= 2):
                        stable = (all(r["initialized"] == 1 and r["status"] == latest["status"] for r in rows)
                                  and latest["status"] in (1, 2)
                                  and max(r["position"] for r in rows)-min(r["position"] for r in rows) <= 2)
                        positions_match = (expected_position is None or all(
                            abs(r["position"]-expected_position) <= position_tolerance for r in rows))
                        if positions_match and (stable_samples == 1 or stable):
                            return {"state": latest, "samples": rows, "stable": stable,
                                    "age_seconds": now-latest["sampled_monotonic"]}
                remaining = deadline-now
                if remaining <= 0:
                    raise RuntimeError("No fresh gripper feedback satisfying the requested stability window")
                self.condition.wait(min(.1, remaining))

    def close(self):
        self.closed.set()
        with self.condition:
            self.condition.notify_all()
        self.thread.join(timeout=2)
