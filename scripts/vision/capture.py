"""Capture D455 RGB and aligned depth to the project's captures directory."""

import argparse
import base64
from datetime import datetime
import json
from pathlib import Path


def capture_direct():
    import cv2
    import numpy as np
    import pyrealsense2 as rs
    folder = Path(__file__).resolve().parents[2] / "captures"
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_device("141322252639")
    from scripts.vision.rgbd import enable_streams, encode_depth
    enable_streams(config, rs)
    started = False
    try:
        profile = pipeline.start(config)
        started = True
        align = rs.align(rs.stream.color)
        from scripts.vision.camera_settings import apply_rgb_settings
        apply_rgb_settings(profile, rs)
        # Warm up for two seconds at 15 Hz.
        for _ in range(30):
            frames = pipeline.wait_for_frames(5000)
        frame = frames.get_color_frame()
        if not frame:
            raise RuntimeError("Camera returned no RGB frame")
        folder.mkdir(parents=True, exist_ok=True)
        path = folder / (datetime.now().astimezone().strftime("%Y%m%d-%H%M%S-%f%z") + ".jpg")
        if not cv2.imwrite(str(path), np.asanyarray(frame.get_data()),
                           [cv2.IMWRITE_JPEG_QUALITY, 95]):
            raise RuntimeError(f"Could not save image: {path}")
        depth = encode_depth(frames, align)
        save_depth(path, depth)
        path.with_suffix(".json").write_text(json.dumps({
            "frame": {"timestamp_ms": frame.get_timestamp(),
                      "clock_domain": str(frame.get_frame_timestamp_domain()),
                      "frame_number": frame.get_frame_number()},
            "depth": depth}, indent=2))
        print(path)
        return 0
    except Exception as exc:
        print(f"Capture failed: {exc}")
        return 1
    finally:
        if started:
            pipeline.stop()


def save_depth(image_path, depth):
    """Remove encoded payload from metadata and persist unchanged uint16 PNG."""
    path = image_path.with_name(image_path.stem + "-depth.png")
    data = base64.b64decode(depth.pop("png_base64"), validate=True)
    with path.open("xb") as stream:
        stream.write(data)
    depth["image_path"] = str(path)
    return path


def capture_resident(*, client=None, web_url="http://127.0.0.1:8080", after=0.):
    """Save a fresh resident frame and return its paths and original metadata."""
    if client is None:
        from scripts.hardware.sensor_client import SensorClient
        client = SensorClient(web_url)
    packet = dict(client.capture(after=after))
    image = base64.b64decode(packet.pop("png_base64"), validate=True)
    folder = Path(__file__).resolve().parents[2] / "captures"
    folder.mkdir(parents=True, exist_ok=True)
    path = folder / (datetime.now().astimezone().strftime("%Y%m%d-%H%M%S-%f%z") + ".png")
    with path.open("xb") as stream:
        stream.write(image)
    depth_path = save_depth(path, packet["depth"])
    with path.with_suffix(".json").open("x") as stream:
        json.dump(packet, stream, indent=2, allow_nan=False)
    return {"image_path": str(path), "metadata_path": str(path.with_suffix(".json")),
            "depth_path": str(depth_path),
            "metadata": packet}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--resident", action="store_true", help="Reuse the web camera; no direct fallback")
    parser.add_argument("--web-url", default="http://127.0.0.1:8080")
    parser.add_argument("--after", type=float, default=0., help="Local monotonic timestamp after robot settling")
    args = parser.parse_args()
    if not args.resident:
        if args.after:
            parser.error("--after requires --resident")
        return capture_direct()
    try:
        print(capture_resident(web_url=args.web_url, after=args.after)["image_path"])
        return 0
    except Exception as exc:
        print(f"Capture failed: {exc}")
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
