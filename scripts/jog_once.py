"""Bounded SDK joint jog. Preview by default; use on an idle robot only.

Software stop is best effort. The physical emergency stop remains necessary.
SDK disconnect can stop motion even in preview mode.
"""

import argparse
from datetime import datetime
import json
import math
import os
from pathlib import Path
import sys
import time

ROOT = Path(__file__).resolve().parent.parent
DEFAULT_JOG_RATE = 0.10


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--joint", type=int, choices=range(1, 7), required=True)
    destination = parser.add_mutually_exclusive_group(required=True)
    destination.add_argument("--delta-deg", type=float)
    destination.add_argument("--target-deg", type=float)
    destination.add_argument("--restore-initial", action="store_true")
    parser.add_argument("--execute", action="store_true")
    args = parser.parse_args()
    max_delta = 5.0 if args.joint == 6 else 0.1
    if args.target_deg is not None and (args.joint != 6 or not math.isfinite(args.target_deg)):
        parser.error("absolute targets are supported only for J6 and must be finite")
    if args.delta_deg is not None and (not math.isfinite(args.delta_deg) or not 0.01 <= abs(args.delta_deg) <= max_delta):
        parser.error(f"absolute delta must be finite and between 0.01 and {max_delta} degrees")
    if args.restore_initial:
        baseline_path = ROOT / "config" / "initial_joint_pose.json"
        initial = json.loads(baseline_path.read_text(encoding="utf-8"))["joint_deg"]
        if len(initial) != 6 or not all(math.isfinite(v) for v in initial):
            parser.error("Invalid initial position log")
        args.target_deg = initial[args.joint - 1]

    lib = ROOT / "vendor/xCoreSDK-Python/Release/windows"
    log_dir = ROOT / "logs" / "runs"
    sdk_dir = ROOT / "logs" / "sdk"
    log_dir.mkdir(parents=True, exist_ok=True)
    sdk_dir.mkdir(parents=True, exist_ok=True)
    # Native SDK logs are relative to the process working directory.
    os.chdir(sdk_dir)
    log_path = log_dir / ("jog-" + datetime.now().strftime("%Y%m%d-%H%M%S-%f") + ".jsonl")
    with os.add_dll_directory(str(lib)), log_path.open("w", encoding="utf-8") as log:
        sys.path.insert(0, str(lib))
        import xCoreSDK_python as sdk

        def emit(name, **fields):
            line = json.dumps({"time": datetime.now().astimezone().isoformat(),
                               "step": name, **fields}, default=str)
            print(line, flush=True)
            log.write(line + "\n")
            log.flush()

        def call(name, method, *values):
            ec = {}
            result = method(*values, ec)
            emit(name, value=result, error=ec)
            if ec.get("ec") != 0:
                raise RuntimeError(f"{name}: {ec}")
            return result

        robot = sdk.xMateRobot()
        connected = False
        armed = False
        failed = False
        try:
            robot.connectToRobot("192.168.0.160", "")
            connected = True
            info = call("info", robot.robotInfo)
            if info.type != "XMS4-R800-B4S1A2C" or info.joint_num != 6:
                raise RuntimeError("Unexpected robot model")

            def snapshot():
                if call("mode", robot.operateMode) != sdk.OperateMode.manual:
                    raise RuntimeError("Manual mode required; no automatic mode switching")
                if call("state", robot.operationState) != sdk.OperationState.idle:
                    raise RuntimeError("Robot must be idle")
                power = call("power", robot.powerState)
                if power not in (sdk.PowerState.on, sdk.PowerState.off):
                    raise RuntimeError("Robot power/safety state is not ready")
                joints = call("joints", robot.jointPos)[:6]
                if len(joints) != 6 or not all(math.isfinite(v) for v in joints):
                    raise RuntimeError("Invalid joint feedback")
                return joints, power

            start, _ = snapshot()
            target = start.copy()
            if args.target_deg is not None:
                args.delta_deg = args.target_deg - math.degrees(start[args.joint - 1])
                travel_limit = (30 if args.joint == 6 else 1) if args.restore_initial else 20
                if abs(args.delta_deg) > travel_limit:
                    raise RuntimeError(f"Absolute target requires more than {travel_limit} degrees of travel")
                if abs(args.delta_deg) < 0.01:
                    emit("already_at_target", actual_deg=math.degrees(start[args.joint - 1]))
                    return 0
            target[args.joint - 1] += math.radians(args.delta_deg)
            limits = sdk.PyTypeVectorArrayDouble2()
            enabled = call("soft_limits_enabled", robot.getSoftLimit, limits)
            bounds = limits.content()
            if len(bounds) < 6 or any(not (lo <= v <= hi)
                                      for v, (lo, hi) in zip(target, bounds)):
                raise RuntimeError("Target outside reported joint bounds")
            emit("plan", joint=args.joint, delta_deg=args.delta_deg, sdk_rate=DEFAULT_JOG_RATE,
                 start_deg=list(map(math.degrees, start)),
                 target_deg=list(map(math.degrees, target)), soft_limits_enabled=enabled,
                 note="Joint bounds do not check obstacles, fingers or cables")
            if not args.execute:
                emit("preview_only")
                return 0

            confirmation = input("Confirm path clear, load correct, emergency stop ready. Type EXECUTE: ")
            if confirmation != "EXECUTE":
                emit("cancelled")
                return 0
            current, power = snapshot()
            if max(abs(a - b) for a, b in zip(start, current)) > math.radians(0.01):
                raise RuntimeError("Robot changed position; preview again")
            armed = True
            if power == sdk.PowerState.off:
                call("power_on", robot.setPowerState, True)
            if call("power_ready", robot.powerState) != sdk.PowerState.on:
                raise RuntimeError("Power not on; physical enable may be required")
            call("startJog", robot.startJog, sdk.JogOptSpace.jointSpace,
                 DEFAULT_JOG_RATE, abs(args.delta_deg), args.joint - 1, args.delta_deg > 0)
            timeout_seconds = 60 if args.restore_initial else (40 if args.target_deg is not None else 10)
            deadline = time.monotonic() + timeout_seconds
            reached = False
            while time.monotonic() < deadline:
                actual = call("feedback", robot.jointPos)[:6]
                if len(actual) != 6 or not all(math.isfinite(v) for v in actual):
                    raise RuntimeError("Invalid motion feedback")
                if any(not (min(s, t) - math.radians(0.05) <= a <=
                            max(s, t) + math.radians(0.05))
                       for a, s, t in zip(actual, start, target)):
                    raise RuntimeError("Unexpected displacement")
                if call("power_monitor", robot.powerState) != sdk.PowerState.on:
                    raise RuntimeError("Power lost during jog")
                if max(abs(a - b) for a, b in zip(actual, target)) <= math.radians(0.005):
                    reached = True
                    break
                time.sleep(0.1)
            call("stop", robot.stop)
            if not reached:
                raise RuntimeError(f"Target not reached within {timeout_seconds} seconds; no retry")
            emit("target_observed", actual_deg=list(map(math.degrees, actual)))
            samples = []
            for _ in range(5):
                time.sleep(0.2)
                final = call("stopped_joints", robot.jointPos)[:6]
                if (len(final) != 6 or not all(math.isfinite(v) for v in final)
                        or max(abs(a - b) for a, b in zip(final, target)) > math.radians(0.01)):
                    raise RuntimeError("Stopped position verification failed")
                samples.append(final)
            call("stopped_velocity", robot.jointVel)
            if any(max(axis) - min(axis) > math.radians(0.002) for axis in zip(*samples)):
                raise RuntimeError("Position not stable after stop")
            emit("verified_motion", joint=args.joint,
                 delta_deg=[math.degrees(a - b) for a, b in zip(final, start)])
        except (Exception, KeyboardInterrupt) as exc:
            failed = True
            emit("error", message=str(exc), type=type(exc).__name__)
        finally:
            if connected:
                cleanup = []
                if armed:
                    cleanup.extend([("stop_final", robot.stop, ()),
                                    ("power_off", robot.setPowerState, (False,)),
                                    ("power_after", robot.powerState, ())])
                cleanup.append(("disconnect", robot.disconnectFromRobot, ()))
                for name, method, values in cleanup:
                    try:
                        result = call(name, method, *values)
                        if name == "power_after" and result != sdk.PowerState.off:
                            raise RuntimeError("Power off not confirmed")
                    except Exception as exc:
                        failed = True
                        emit("cleanup_failed", action=name, message=str(exc))
            print(f"Log: {log_path}")
            if failed:
                raise SystemExit(1)
        return int(failed)


if __name__ == "__main__":
    raise SystemExit(main())
