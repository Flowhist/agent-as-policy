"""Apply the saved RGB exposure without changing stream geometry."""

import json
from pathlib import Path


def apply_rgb_settings(profile, rs):
    path = Path(__file__).resolve().parents[2] / "config/camera_rgb.json"
    settings = json.loads(path.read_text())
    device = profile.get_device()
    if device.get_info(rs.camera_info.serial_number) != settings["serial"]:
        raise RuntimeError("RGB settings are for a different camera")
    for sensor in device.query_sensors():
        if sensor.get_info(rs.camera_info.name) == "RGB Camera":
            sensor.set_option(rs.option.enable_auto_exposure, float(settings["enable_auto_exposure"]))
            if not settings["enable_auto_exposure"]:
                sensor.set_option(rs.option.gain, settings["gain"])
                sensor.set_option(rs.option.exposure, settings["exposure"])
            return
    raise RuntimeError("RGB sensor not found")
