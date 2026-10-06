import importlib.util
import math
from pathlib import Path
import tempfile
import unittest
import base64
import io
import json
from unittest.mock import patch, MagicMock

import cv2
import yaml

spec = importlib.util.spec_from_file_location(
    "calibration_capture", Path(__file__).resolve().parents[1] / "scripts/calibration_capture.py")
capture = importlib.util.module_from_spec(spec)
spec.loader.exec_module(capture)


class CaptureTests(unittest.TestCase):
    def test_web_capture_writes_paired_png_and_appends_yaml(self):
        a = cv2.aruco
        board = a.CharucoBoard((12, 9), .015, .01125,
                              a.getPredefinedDictionary(a.DICT_5X5_1000))
        _, png = cv2.imencode('.png', board.generateImage((1200, 900), marginSize=30))
        reading = {"joints_rad": [0.] * 6, "flange_in_base_m_rad": [0.] * 6}
        packet = {"camera": {"serial": "test"}, "png_base64": base64.b64encode(png).decode(),
                  "robot_readings": [reading] * 4, "frame": {"frame_number": 1}, "robot": {}}
        opener = MagicMock()
        def response(req, **kwargs):
            data = {"token": "test"} if req.full_url.endswith('/session') else packet
            return io.BytesIO(json.dumps(data).encode())
        opener.open.side_effect = response
        with tempfile.TemporaryDirectory() as folder:
            with patch.object(capture.urllib.request, 'build_opener', return_value=opener):
                capture.capture_web(Path(folder), 'train')
                capture.capture_web(Path(folder), 'validation')
            data = yaml.safe_load((Path(folder) / 'samples.yaml').read_text())
            self.assertEqual([s['split'] for s in data['samples']], ['train', 'validation'])
            for sample in data['samples']:
                self.assertEqual(sample['detection']['corner_count'], 88)
                self.assertTrue((Path(folder) / sample['image']).is_file())
                self.assertEqual(len(sample['robot_readings']), 4)

    def test_camera_start_failure_is_logged_without_robot_connection(self):
        pipeline = MagicMock()
        pipeline.start.side_effect = RuntimeError("Device or resource busy")
        with tempfile.TemporaryDirectory() as folder:
            with patch("pyrealsense2.pipeline", return_value=pipeline), \
                    patch.object(capture.subprocess, "Popen") as popen:
                with self.assertRaisesRegex(RuntimeError, "Close web camera preview"):
                    capture.capture(Path(folder), "auto")
                popen.assert_not_called()
            logs = list(Path(folder).glob("*-sdk.log"))
            self.assertEqual(len(logs), 1)
            self.assertIn("Device or resource busy", logs[0].read_text())
            self.assertIn("FAILED", logs[0].read_text())
            self.assertFalse((Path(folder) / "samples.yaml").exists())

    def test_motion_rejected(self):
        a = {"joints_rad": [0.] * 6, "flange_in_base_m_rad": [0.] * 6}
        b = {"joints_rad": [math.radians(0.02)] + [0.] * 5,
             "flange_in_base_m_rad": [0.] * 6}
        with self.assertRaises(RuntimeError):
            capture.stable([a, b])
        b["joints_rad"] = [0.] * 6
        b["flange_in_base_m_rad"][0] = 0.001
        with self.assertRaises(RuntimeError):
            capture.stable([a, b])
        self.assertEqual(capture.stable([a, a])["max_joint_change_deg"], 0)

    def test_board_detection_and_blank_rejection(self):
        import numpy as np
        a = cv2.aruco
        board = a.CharucoBoard((12, 9), .015, .01125,
                              a.getPredefinedDictionary(a.DICT_5X5_1000))
        image = board.generateImage((1200, 900), marginSize=30)
        detection, _ = capture.detect(image)
        self.assertEqual(detection["corner_count"], 88)
        with self.assertRaises(RuntimeError):
            capture.detect(np.full((720, 1280), 255, dtype=np.uint8))

    def test_yaml_preserves_previous_samples(self):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / "samples.yaml"
            data = {"samples": [{"id": "first"}]}
            capture.atomic_yaml(path, data)
            loaded = yaml.safe_load(path.read_text())
            loaded["samples"].append({"id": "second"})
            capture.atomic_yaml(path, loaded)
            self.assertEqual(yaml.safe_load(path.read_text()), loaded)
            self.assertEqual(list(Path(folder).glob("*.tmp")), [])


if __name__ == "__main__":
    unittest.main()
