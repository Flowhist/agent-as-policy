"""Capture one D455 RGB image to the project's captures directory."""

from datetime import datetime
from pathlib import Path

import cv2
import numpy as np
import pyrealsense2 as rs


def main():
    folder = Path(__file__).resolve().parent.parent / "captures"
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_device("141322252639")
    config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 10)
    started = False
    try:
        profile = pipeline.start(config)
        started = True
        from camera_settings import apply_rgb_settings
        apply_rgb_settings(profile, rs)
        # Warm up for about three seconds at the experiment's 10 Hz rate.
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
        print(path)
        return 0
    except Exception as exc:
        print(f"Capture failed: {exc}")
        return 1
    finally:
        if started:
            pipeline.stop()


if __name__ == "__main__":
    raise SystemExit(main())
