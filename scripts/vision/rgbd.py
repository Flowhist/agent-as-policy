"""Shared D455 stream configuration and lossless RGB-aligned depth encoding."""


def enable_streams(config, rs):
    config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 15)
    config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 15)


def encode_depth(frames, align):
    import base64
    import cv2
    import numpy as np

    depth = align.process(frames).get_depth_frame()
    if not depth:
        raise RuntimeError("Camera returned no RGB-aligned depth frame")
    pixels = np.asanyarray(depth.get_data())
    ok, png = cv2.imencode(".png", pixels)
    if not ok:
        raise RuntimeError("Depth PNG encoding failed")
    return {"png_base64": base64.b64encode(png.tobytes()).decode(),
            "aligned_to": "color", "encoding": "uint16_png",
            "width": depth.get_width(), "height": depth.get_height(),
            "meters_per_unit": depth.get_units(), "invalid_value": 0,
            "source_fps": 15,
            "timestamp_ms": depth.get_timestamp(),
            "clock_domain": str(depth.get_frame_timestamp_domain()),
            "frame_number": depth.get_frame_number()}
