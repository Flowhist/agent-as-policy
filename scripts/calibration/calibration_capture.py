"""Capture one stationary eye-to-hand sample; never commands robot motion."""

import argparse
import base64
from datetime import datetime, timezone
import fcntl
import json
import math
import os
from pathlib import Path
import socket
import subprocess
import sys
import tempfile
import time
import urllib.request
import urllib.error

ROOT = Path(__file__).resolve().parents[2]
BOARD = {"squares_x": 12, "squares_y": 9, "square_length_m": 0.015,
         "marker_length_m": 0.01125, "dictionary": "DICT_5X5_1000",
         "marker_ids": list(range(54)), "legacy_pattern": False,
         "note": "5x5 dictionary superset; observed IDs/layout checked against printed board"}
SERIAL = "141322252639"


def utc():
    return datetime.now(timezone.utc).isoformat()


def robot_worker(fd):
    # Keep xCore native libraries separate from RealSense and OpenCV.
    from scripts.hardware.sdk_runtime import sdk_runtime
    channel = socket.socket(fileno=fd).makefile("rw")
    robot = None
    connected = False
    def send(value):
        channel.write(json.dumps(value) + "\n")
        channel.flush()
    def call(method, *args):
        ec = {}
        value = method(*args, ec)
        if ec.get("ec") != 0:
            raise RuntimeError(f"{method.__name__}: {ec}")
        return value
    try:
        with sdk_runtime():
            import xCoreSDK_python as sdk
            os.chdir(ROOT / "logs" / "sdk")
            try:
                robot = sdk.xMateRobot()
                robot.connectToRobot("192.168.0.160", "")
                connected = True
                info = call(robot.robotInfo)
                if info.type != "XMS4-R800-B4S1A2C" or info.joint_num != 6:
                    raise RuntimeError("Unexpected robot model")
                send({"ready": True, "model": info.type, "sdk": sdk.BaseRobot.sdkVersion()})
                for line in channel:
                    if json.loads(line) == "quit":
                        break
                    start = utc()
                    mode = call(robot.operateMode)
                    operation = call(robot.operationState)
                    power = call(robot.powerState)
                    if mode != sdk.OperateMode.manual or operation != sdk.OperationState.idle:
                        raise RuntimeError("Exit drag first: robot must be manual and idle")
                    if power not in (sdk.PowerState.on, sdk.PowerState.off):
                        raise RuntimeError("Robot power/safety state is not ready")
                    joints = list(call(robot.jointPos))[:6]
                    pose = list(call(robot.posture, sdk.CoordinateType.flangeInBase))
                    if len(joints) != 6 or len(pose) != 6 or not all(map(math.isfinite, joints + pose)):
                        raise RuntimeError("Invalid robot feedback")
                    matrix = list(sdk.utility.postureToTransArray(pose))
                    if len(matrix) != 16 or not all(map(math.isfinite, matrix)) or abs(matrix[15] - 1) > 1e-9:
                        raise RuntimeError("Invalid SDK transformation matrix")
                    send({"read_start_utc": start, "read_end_utc": utc(),
                          "joints_rad": joints, "flange_in_base_m_rad": pose,
                          "T_base_flange_row_major": matrix,
                          "mode": str(mode), "operation": str(operation), "power": str(power)})
            finally:
                if connected:
                    call(robot.disconnectFromRobot)
        send({"closed": True})
    except Exception as exc:
        send({"error": str(exc)})
    finally:
        channel.close()


def stable(readings):
    first = readings[0]
    max_joint = max(abs(a - b) for r in readings for a, b in
                    zip(first["joints_rad"], r["joints_rad"]))
    max_translation = max(math.dist(first["flange_in_base_m_rad"][:3],
                                    r["flange_in_base_m_rad"][:3]) for r in readings)
    if max_joint > math.radians(0.01) or max_translation > 0.0002:
        raise RuntimeError("Robot moved during capture; sample rejected (limits: 0.01 deg, 0.2 mm)")
    return {"max_joint_change_deg": math.degrees(max_joint),
            "max_flange_translation_m": max_translation}


def detect(image):
    import cv2
    import numpy as np
    aruco = cv2.aruco
    board = aruco.CharucoBoard((12, 9), 0.015, 0.01125,
                              aruco.getPredefinedDictionary(aruco.DICT_5X5_1000))
    board.setLegacyPattern(False)
    corners, ids, markers, marker_ids = aruco.CharucoDetector(board).detectBoard(image)
    count = 0 if ids is None else len(ids)
    if count < 20:
        raise RuntimeError(f"Only {count} ChArUco corners; need at least 20")
    obj = board.getChessboardCorners()[ids.flatten()]
    span = np.ptp(obj[:, :2], axis=0)
    if span[0] < 0.075 or span[1] < 0.06:
        raise RuntimeError("Detected corners cover too little of the board")
    annotated = image.copy()
    aruco.drawDetectedCornersCharuco(annotated, corners, ids)
    return {"charuco_ids": ids.flatten().tolist(), "corners_px": corners.reshape(-1, 2).tolist(),
            "object_points_board_m": obj.tolist(), "corner_count": count,
            "marker_ids": marker_ids.flatten().tolist(),
            "marker_corners_px": [v.reshape(-1, 2).tolist() for v in markers]}, annotated


def atomic_yaml(path, data):
    import yaml
    fd, name = tempfile.mkstemp(prefix=".samples-", suffix=".tmp", dir=path.parent)
    try:
        with os.fdopen(fd, "w") as stream:
            yaml.safe_dump(data, stream, allow_unicode=True, sort_keys=False)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(name, path)
    finally:
        if os.path.exists(name):
            os.unlink(name)


def capture_web(folder, split):
    import cv2
    import numpy as np
    import yaml
    folder.mkdir(parents=True, exist_ok=True)
    with (folder / ".capture.lock").open("a") as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        path = folder / "samples.yaml"
        dataset = yaml.safe_load(path.read_text()) if path.exists() else {
            "schema_version": 1, "created_utc": utc(), "board": BOARD,
            "frame_convention": "T_A_B maps coordinates from B to A; SDK matrix is row-major", "samples": []}
        if dataset.get("schema_version") != 1 or dataset.get("board") != BOARD:
            raise RuntimeError("Dataset configuration differs; use a new --session")
        stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S%fZ")
        log_path = folder / (stamp + "-capture.log")
        with log_path.open("w") as log:
            def post(route, data, token=None):
                headers = {"Content-Type": "application/json"}
                if token:
                    headers["Authorization"] = "Bearer " + token
                req = urllib.request.Request("http://127.0.0.1:8080" + route,
                                             data=json.dumps(data).encode(), headers=headers)
                try:
                    with urllib.request.build_opener(urllib.request.ProxyHandler({})).open(req, timeout=20) as response:
                        return json.load(response)
                except urllib.error.HTTPError as exc:
                    raise RuntimeError(f"Web service {exc.code}: {exc.read().decode()}. "
                                       "If endpoint is missing, restart the updated web service while idle.") from exc
            try:
                log.write(f"{utc()} Requesting sample from web service; no device reconnect\n")
                log.flush()
                token = post("/api/session", {})["token"]
                packet = post("/api/calibration/capture", {}, token)
                if "camera" in dataset and dataset["camera"] != packet["camera"]:
                    raise RuntimeError("Camera intrinsics changed; use a new --session")
                png = base64.b64decode(packet["png_base64"], validate=True)
                image = cv2.imdecode(np.frombuffer(png, dtype=np.uint8), cv2.IMREAD_COLOR)
                if image is None:
                    raise RuntimeError("Invalid captured PNG")
                quality = stable(packet["robot_readings"])
                detection, annotated = detect(image)
                image_name, overlay_name = stamp + ".png", stamp + "-corners.png"
                (folder / image_name).write_bytes(png)
                if not cv2.imwrite(str(folder / overlay_name), annotated):
                    raise RuntimeError("Cannot save corner preview")
                count = sum(s["split"] == "train" for s in dataset["samples"])
                sample_split = split if split != "auto" else ("train" if count < 20 else "validation")
                dataset["camera"] = packet["camera"]
                dataset["samples"].append({"id": stamp, "split": sample_split, "saved_utc": utc(),
                    "image": image_name, "overlay": overlay_name, "frame": packet["frame"],
                    "robot": packet["robot"], "robot_readings": packet["robot_readings"],
                    "stability": quality, "detection": detection})
                atomic_yaml(path, dataset)
                counts = {k: sum(s["split"] == k for s in dataset["samples"]) for k in ("train", "validation")}
                message = f"Capture complete: {detection['corner_count']}/88 corners; {counts}\nYAML: {path}"
                log.write(message + "\n")
                print(message)
            except BaseException as exc:
                log.write(f"{utc()} FAILED: {type(exc).__name__}: {exc}\n")
                print(f"Failure log: {log_path}", file=sys.stderr)
                raise


def capture(folder, split):
    import cv2
    import numpy as np
    import pyrealsense2 as rs
    import yaml
    folder.mkdir(parents=True, exist_ok=True)
    path = folder / "samples.yaml"
    with (folder / ".capture.lock").open("a") as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        dataset = yaml.safe_load(path.read_text()) if path.exists() else {
            "schema_version": 1, "created_utc": utc(), "board": BOARD,
            "frame_convention": "T_A_B maps coordinates from B to A; SDK matrix is row-major",
            "samples": []}
        if dataset.get("schema_version") != 1 or dataset.get("board") != BOARD:
            raise RuntimeError("Dataset configuration differs; use a new --session directory")
        stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S%fZ")
        pipeline = rs.pipeline()
        config = rs.config()
        config.enable_device(SERIAL)
        config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 10)
        # Camera opens first so an occupied camera never causes a robot connection.
        started = False
        process = None
        parent, child = socket.socketpair()
        parent.settimeout(15)
        channel = parent.makefile("rw")
        log = (folder / (stamp + "-sdk.log")).open("w")
        stage = "initializing"
        def progress(message):
            nonlocal stage
            stage = message
            line = f"{utc()} {message}"
            print(line, flush=True)
            log.write(line + "\n")
            log.flush()
        def receive():
            line = channel.readline()
            if not line:
                raise RuntimeError("Robot worker exited; inspect SDK log")
            value = json.loads(line)
            if "error" in value:
                raise RuntimeError(value["error"])
            return value
        def request(command):
            channel.write(json.dumps(command) + "\n")
            channel.flush()
            return receive()
        try:
            progress("Starting camera (close web camera preview first)")
            try:
                profile = pipeline.start(config)
            except RuntimeError as exc:
                raise RuntimeError(
                    "Camera start failed. Close web camera preview and other camera apps, "
                    "then check the D455 USB connection. Original error: " + str(exc)) from exc
            started = True
            from scripts.vision.camera_settings import apply_rgb_settings
            apply_rgb_settings(profile, rs)
            progress("Warming up camera for 30 frames")
            for _ in range(30):
                pipeline.wait_for_frames(2000)
            intr = profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
            camera = {"serial": SERIAL, "width": intr.width, "height": intr.height, "fps": 10,
                      "fx": intr.fx, "fy": intr.fy, "ppx": intr.ppx, "ppy": intr.ppy,
                      "distortion_model": str(intr.model), "coefficients": list(intr.coeffs),
                      "note": "Raw RGB; do not treat inverse_brown_conrady as OpenCV Brown distortion"}
            if "camera" in dataset and dataset["camera"] != camera:
                raise RuntimeError("Camera intrinsics changed; use a new --session directory")
            (ROOT / "logs" / "sdk").mkdir(parents=True, exist_ok=True)
            progress("Connecting robot worker (web robot must be disconnected)")
            process = subprocess.Popen([str(ROOT / ".venv/bin/python"), "-m", "scripts.calibration.calibration_capture",
                                        "--robot-worker", str(child.fileno())],
                                       pass_fds=(child.fileno(),), stdout=log, stderr=log, cwd=ROOT)
            child.close()
            robot_info = receive()
            progress("Reading stationary robot pose and capturing a fresh frame")
            readings = [request("read")]
            time.sleep(0.5)
            readings.append(request("read"))
            stable(readings)
            capture_start = utc()
            # Discard queued frames, then span multiple live frame periods inside the pose bracket.
            pipeline.wait_for_frames(2000)
            for _ in range(3):
                frames = pipeline.wait_for_frames(2000)
            frame = frames.get_color_frame()
            if not frame:
                raise RuntimeError("No RGB frame")
            image = np.asanyarray(frame.get_data()).copy()
            frame_meta = {"timestamp_ms": frame.get_timestamp(), "clock_domain": str(frame.get_frame_timestamp_domain()),
                          "frame_number": frame.get_frame_number(), "host_capture_start_utc": capture_start,
                          "host_received_utc": utc()}
            readings.append(request("read"))
            time.sleep(0.2)
            readings.append(request("read"))
            quality = stable(readings)
            if request("quit") != {"closed": True}:
                raise RuntimeError("Robot disconnect not confirmed")
            process.wait(timeout=5)
            progress("Robot disconnected; detecting board corners")
            detection, annotated = detect(image)
            image_name, overlay_name = stamp + ".png", stamp + "-corners.png"
            for name, pixels in [(image_name, image), (overlay_name, annotated)]:
                if not cv2.imwrite(str(folder / name), pixels):
                    raise RuntimeError(f"Cannot save {name}")
            train_count = sum(s["split"] == "train" for s in dataset["samples"])
            sample_split = split if split != "auto" else ("train" if train_count < 20 else "validation")
            sample = {"id": stamp, "split": sample_split, "saved_utc": utc(), "image": image_name,
                      "overlay": overlay_name, "frame": frame_meta, "robot": robot_info,
                      "robot_readings": readings, "stability": quality, "detection": detection}
            dataset["camera"] = camera
            dataset["samples"].append(sample)
            progress("Saving images and samples.yaml")
            atomic_yaml(path, dataset)
            counts = {k: sum(s["split"] == k for s in dataset["samples"]) for k in ("train", "validation")}
            print(f"Saved {sample_split}: {detection['corner_count']} corners; {counts}")
            print(f"YAML: {path}\nOverlay: {folder / overlay_name}")
            progress("Capture complete")
        except BaseException as exc:
            log.write(f"{utc()} FAILED at {stage}: {type(exc).__name__}: {exc}\n")
            log.flush()
            print(f"Failure log: {log.name}", file=sys.stderr, flush=True)
            raise
        finally:
            # EOF asks the worker to disconnect; never leave a background SDK owner.
            try:
                parent.shutdown(socket.SHUT_WR)
            except OSError:
                pass
            if process is not None and process.poll() is None:
                try:
                    process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait()
                    print("WARNING: SDK worker blocked; disconnect unverified. Check robot locally.", file=sys.stderr)
            channel.close()
            parent.close()
            child.close()
            log.close()
            if started:
                pipeline.stop()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--session", type=Path, default=ROOT / "captures" / "calibration" / "session-01",
                        help="New directory whenever camera, robot base or board mounting changes")
    parser.add_argument("--split", choices=("auto", "train", "validation"), default="auto")
    parser.add_argument("--direct", action="store_true", help="Legacy exclusive-device capture; web devices must be disconnected")
    parser.add_argument("--check-image", type=Path, help="Offline detection only; no device access")
    parser.add_argument("--robot-worker", type=int, help=argparse.SUPPRESS)
    args = parser.parse_args()
    if args.robot_worker is not None:
        robot_worker(args.robot_worker)
        return
    try:
        if args.check_image:
            import cv2
            image = cv2.imread(str(args.check_image))
            if image is None:
                raise RuntimeError("Cannot read image")
            result, _ = detect(image)
            print(f"Detected {result['corner_count']}/88 corners; no hardware accessed")
        else:
            (capture if args.direct else capture_web)(args.session.resolve(), args.split)
    except Exception as exc:
        print(f"Capture rejected: {exc}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
