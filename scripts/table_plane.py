"""Estimate a table plane from a parallel raised board; offline only."""

import argparse
from pathlib import Path
import hashlib

import cv2
import numpy as np
import pyrealsense2 as rs
import yaml

from calibration_capture import detect
from calibration_solve import transform, points


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--image', type=Path, required=True)
    parser.add_argument('--session', type=Path, required=True)
    parser.add_argument('--height-mm', type=float, required=True)
    args = parser.parse_args()
    if not 0 < args.height_mm < 2000:
        raise ValueError('Invalid total board-pattern height')
    data = yaml.safe_load((args.session / 'samples.yaml').read_text())
    extrinsic_path = args.session / 'solve-15-validate-5/result.yaml'
    result = yaml.safe_load(extrinsic_path.read_text())
    camera = data['camera']
    intr = rs.intrinsics()
    for key in ('width', 'height', 'fx', 'fy', 'ppx', 'ppy'):
        setattr(intr, key, camera[key])
    if camera['distortion_model'] != 'distortion.inverse_brown_conrady':
        raise ValueError('Unexpected distortion model')
    intr.model = rs.distortion.inverse_brown_conrady
    intr.coeffs = camera['coefficients']
    image = cv2.imread(str(args.image))
    if image is None or image.shape[:2] != (intr.height, intr.width):
        raise ValueError('Image missing or wrong resolution')
    detection, _ = detect(image)
    obj = np.array(detection['object_points_board_m'], dtype=np.float64)
    pixels = detection['corners_px']
    rays = np.array([rs.rs2_deproject_pixel_to_point(intr, xy, 1.) for xy in pixels])
    normalized = rays[:, :2] / rays[:, 2:3]
    ok, rv, tv = cv2.solvePnP(obj, normalized, np.eye(3), None, flags=cv2.SOLVEPNP_ITERATIVE)
    if not ok:
        raise RuntimeError('PnP failed')
    cb = transform(cv2.Rodrigues(rv)[0], tv)
    bc = np.array(result['T_base_camera'])
    bb = bc @ cb
    normal = bb[:3, 2].copy()
    # The camera is above the table: choose the visible, camera-facing normal.
    if np.dot(normal, bc[:3, 3] - bb[:3, 3]) < 0:
        normal *= -1
    table_point = bb[:3, 3] - args.height_mm / 1000 * normal
    offset = -float(normal @ table_point)
    xyz = points(cb, obj)
    error = (xyz[:, :2] / xyz[:, 2:3] - normalized) * [intr.fx, intr.fy]
    output = {'status': 'provisional_geometry_not_motion_validated',
              'image': str(args.image.resolve()), 'image_sha256': hashlib.sha256(args.image.read_bytes()).hexdigest(),
              'extrinsics': str(extrinsic_path.resolve()),
              'height_pattern_above_table_mm': args.height_mm,
              'assumptions': ['Board parallel to table', 'Camera and base unchanged since calibration',
                              'Camera is on the above-table side', 'Raised board supported rigidly'],
              'corners': len(obj),
              'pnp_rms_ideal_px': float(np.sqrt(np.mean(np.sum(error**2, axis=1)))),
              'T_base_board': bb.tolist(), 'T_camera_board': cb.tolist(),
              'table_point_base_m': table_point.tolist(), 'table_up_normal_base': normal.tolist(),
              'plane_equation': 'n_x*x + n_y*y + n_z*z + d = 0; x,y,z,d in meters',
              'plane_d_m': offset,
              'normal_angle_to_base_positive_z_deg': float(np.degrees(np.arccos(np.clip(normal[2], -1, 1)))),
              'limitations': 'Single JPEG preview frame; assumes stated height and parallelism. Not independently checked.'}
    dest = args.session / 'table-plane.yaml'
    dest.write_text(yaml.safe_dump(output, sort_keys=False))
    print(yaml.safe_dump(output, sort_keys=False))
    print(dest)


if __name__ == '__main__':
    main()
