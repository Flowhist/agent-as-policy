"""Offline eye-to-hand solve: first 15 samples fit, last 5 held out."""

import argparse
import hashlib
import json
from pathlib import Path

import cv2
import numpy as np
import pyrealsense2 as rs
import yaml
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation


def transform(rotation, translation):
    t = np.eye(4)
    t[:3, :3] = rotation
    t[:3, 3] = np.asarray(translation).reshape(3)
    return t


def unpack(v):
    return transform(Rotation.from_rotvec(v[:3]).as_matrix(), v[3:6])


def pack(t):
    return np.r_[Rotation.from_matrix(t[:3, :3]).as_rotvec(), t[:3, 3]]


def points(t, p):
    return p @ t[:3, :3].T + t[:3, 3]


def initialize(b, c):
    # OpenCV solves A*U=V*D. Here A=B_i, D=C_i, U=Y, V=X:
    # T_base_flange * T_flange_board = T_base_camera * T_camera_board.
    ry, ty, rx, tx = cv2.calibrateRobotWorldHandEye(
        [t[:3, :3] for t in b], [t[:3, 3] for t in b],
        [t[:3, :3] for t in c], [t[:3, 3] for t in c],
        method=cv2.CALIB_ROBOT_WORLD_HAND_EYE_SHAH)
    return transform(rx, tx), transform(ry, ty)


def self_test():
    rng = np.random.default_rng(17)
    x = unpack([.2, -.4, .1, .5, -.3, .8])
    y = unpack([-.3, .1, .2, .02, .03, .2])
    b = [unpack(np.r_[rng.normal(size=3), rng.normal(size=3)]) for _ in range(15)]
    c = [np.linalg.inv(x) @ t @ y for t in b]
    xx, yy = initialize(b, c)
    assert np.max(np.abs(xx - x)) < 1e-8
    assert np.max(np.abs(yy - y)) < 1e-8
    print('Synthetic transform-direction recovery passed')


def main(path):
    raw = path.read_bytes()
    data = yaml.safe_load(raw)
    samples = data['samples']
    if len(samples) != 20:
        raise ValueError('Expected exactly 20 samples for the fixed 15/5 split')
    cam = data['camera']
    intr = rs.intrinsics()
    for key in ('width', 'height', 'fx', 'fy', 'ppx', 'ppy'):
        setattr(intr, key, cam[key])
    if cam['distortion_model'] != 'distortion.inverse_brown_conrady':
        raise ValueError('This solver expects the recorded inverse Brown model')
    intr.model = rs.distortion.inverse_brown_conrady
    intr.coeffs = cam['coefficients']
    focal = np.array([intr.fx, intr.fy])
    b, c, observations, objects, checks, tiles = [], [], [], [], [], []
    for i, sample in enumerate(samples):
        image = cv2.imread(str(path.parent / sample['image']))
        if image is None or image.shape[:2] != (intr.height, intr.width):
            raise ValueError(f'Image missing or wrong size: sample {i+1}')
        # Re-detect from raw images so saved corner arrays are independently checked.
        from calibration_capture import detect, stable
        detection, overlay = detect(image)
        stable(sample['robot_readings'])
        if detection['charuco_ids'] != sample['detection']['charuco_ids']:
            raise ValueError(f'Corner ID mismatch: sample {i+1}')
        p = np.asarray(detection['object_points_board_m'], dtype=np.float64)
        uv = np.asarray(detection['corners_px'], dtype=np.float64)
        rays = np.array([rs.rs2_deproject_pixel_to_point(intr, xy.tolist(), 1.) for xy in uv])
        normalized = rays[:, :2] / rays[:, 2:3]
        ok, rvec, tvec = cv2.solvePnP(p, normalized, np.eye(3), None, flags=cv2.SOLVEPNP_ITERATIVE)
        if not ok:
            raise ValueError(f'PnP failed: {i+1}')
        pose = transform(cv2.Rodrigues(rvec)[0], tvec)
        xyz = points(pose, p)
        if np.any(xyz[:, 2] <= 0):
            raise ValueError('Board behind camera')
        error = (xyz[:, :2] / xyz[:, 2:3] - normalized) * focal
        readings = sample['robot_readings']
        bt = np.asarray(readings[2]['T_base_flange_row_major']).reshape(4, 4)
        if not np.allclose(bt[:3, :3].T @ bt[:3, :3], np.eye(3), atol=1e-6):
            raise ValueError('Invalid robot rotation')
        b.append(bt); c.append(pose); objects.append(p); observations.append(normalized)
        checks.append({'index': i+1, 'id': sample['id'], 'split': 'train' if i < 15 else 'validation',
                       'corners': len(p), 'pnp_rms_ideal_px': float(np.sqrt(np.mean(np.sum(error**2, axis=1))))})
        tile = cv2.resize(overlay, (320, 180))
        cv2.putText(tile, f'{i+1:02d} ' + ('FIT' if i<15 else 'HOLDOUT'), (8, 22),
                    cv2.FONT_HERSHEY_SIMPLEX, .6, (0, 0, 255), 2)
        tiles.append(tile)
    x0, y0 = initialize(b[:15], c[:15])
    def residual(v, indices):
        x, y = unpack(v[:6]), unpack(v[6:])
        values = []
        for i in indices:
            xyz = points(np.linalg.inv(x) @ b[i] @ y, objects[i])
            values.extend(((xyz[:, :2] / xyz[:, 2:3] - observations[i]) * focal).ravel())
        return np.asarray(values)
    fit = least_squares(lambda v: residual(v, range(15)), np.r_[pack(x0), pack(y0)],
                        x_scale='jac', max_nfev=1000, ftol=1e-12, xtol=1e-12, gtol=1e-10)
    if not fit.success:
        raise RuntimeError(fit.message)
    x, y = unpack(fit.x[:6]), unpack(fit.x[6:])
    for i, row in enumerate(checks):
        visual = x @ c[i]
        robot = b[i] @ y
        distances = np.linalg.norm(points(visual, objects[i])-points(robot, objects[i]), axis=1)*1000
        row.update({'board_corner_mean_mm': float(distances.mean()), 'board_corner_max_mm': float(distances.max()),
                    'board_origin_error_mm': float(np.linalg.norm(visual[:3,3]-robot[:3,3])*1000),
                    'rotation_error_deg': float(np.degrees(Rotation.from_matrix(visual[:3,:3].T @ robot[:3,:3]).magnitude())),
                    'joint_fit_rms_ideal_px': float(np.sqrt(np.mean(np.sum(residual(fit.x,[i]).reshape(-1,2)**2,axis=1))))})
    def summary(rows):
        return {'mean_board_corner_error_mm': float(np.mean([r['board_corner_mean_mm'] for r in rows])),
                'max_board_corner_error_mm': max(r['board_corner_max_mm'] for r in rows),
                'max_rotation_error_deg': max(r['rotation_error_deg'] for r in rows),
                'rms_ideal_px': float(np.sqrt(np.mean([r['joint_fit_rms_ideal_px']**2 for r in rows])))}
    axes = [Rotation.from_matrix(b[0][:3,:3].T @ t[:3,:3]).as_rotvec() for t in b[1:15]]
    singular = np.linalg.svd(np.asarray(axes), compute_uv=False)
    report = {'source_yaml': str(path.resolve()), 'source_sha256': hashlib.sha256(raw).hexdigest(),
              'split': 'first 15 fit; last 5 held out; original labels unchanged',
              'method': 'RealSense deprojection + OpenCV PnP + Shah AX=ZB + training-only pixel least squares',
              'pixel_metric': 'Undistorted ideal pixel equivalents; NOT raw distorted pixel RMS',
              'units': 'transforms in meters; rotation matrices; T_A_B maps B coordinates to A',
              'T_base_camera': x.tolist(), 'T_camera_base': np.linalg.inv(x).tolist(),
              'T_flange_board': y.tolist(), 'train': summary(checks[:15]), 'validation': summary(checks[15:]),
              'rotation_excitation_singular_values_rad': singular.tolist(),
              'training_position_span_m': np.ptp(np.array([t[:3,3] for t in b[:15]]), axis=0).tolist(),
              'optimizer_jacobian_singular_values': np.linalg.svd(fit.jac, compute_uv=False).tolist(),
              'samples': checks,
              'limitations': 'Same board and robot feedback; does not independently validate TCP, depth, board scale or mounting rigidity.'}
    report['passes_5mm_holdout_target'] = report['validation']['max_board_corner_error_mm'] <= 5
    output = path.parent / 'solve-15-validate-5'
    output.mkdir(exist_ok=True)
    (output/'result.yaml').write_text(yaml.safe_dump(report, sort_keys=False))
    cv2.imwrite(str(output/'samples.jpg'), np.vstack([np.hstack(tiles[k:k+4]) for k in range(0,20,4)]))
    print(json.dumps({k: report[k] for k in ('train','validation','passes_5mm_holdout_target',
                      'rotation_excitation_singular_values_rad','training_position_span_m')}, indent=2))
    print(output/'result.yaml')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('yaml', nargs='?', type=Path,
                        default=Path(__file__).resolve().parents[1]/'captures/calibration/session-01/samples.yaml')
    parser.add_argument('--self-test', action='store_true')
    args = parser.parse_args()
    if args.self_test:
        self_test()
    else:
        main(args.yaml)
