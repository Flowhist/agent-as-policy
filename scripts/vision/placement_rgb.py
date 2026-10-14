"""Offline RGB rim candidates on a caller-provided plane; never moves hardware."""
import argparse
import itertools
import json
from pathlib import Path

import cv2
import numpy as np
import pyrealsense2 as rs

from scripts.vision.block_top import intersection, load_plane


def plane_points(pixels, calibration, clearance_m):
    rays = np.array([rs.rs2_deproject_pixel_to_point(
        calibration.intr, list(map(float, p)), 1.) for p in pixels]) @ calibration.bc[:3, :3].T
    denominator = rays @ calibration.normal
    if np.any(np.abs(denominator) < 1e-6):
        raise ValueError('Ray parallel to requested plane')
    distance = (clearance_m - calibration.d - calibration.normal @ calibration.bc[:3, 3]) / denominator
    if np.any(distance <= 0):
        raise ValueError('Requested plane behind camera')
    return calibration.bc[:3, 3] + rays * distance[:, None]


def edge_support(corners, distance):
    support = []
    for a, b in zip(corners, np.roll(corners, -1, axis=0)):
        samples = a + np.linspace(.04, .96, 80)[:, None] * (b-a)
        values = cv2.remap(distance, samples[:, 0:1].astype(np.float32),
                           samples[:, 1:2].astype(np.float32), cv2.INTER_LINEAR).ravel()
        support.append(float(np.mean(values <= 2.0)))
    return support


def rim_candidates(image, reference, calibration, rim_m, roi):
    if (image is None or reference is None or image.shape != reference.shape
            or image.shape[:2] != (calibration.intr.height, calibration.intr.width)):
        raise ValueError('Images do not match calibrated resolution')
    x, y, width, height = roi
    if min(x, y) < 0 or min(width, height) <= 0 or x+width > image.shape[1] or y+height > image.shape[0]:
        raise ValueError('Invalid ROI')
    distances = []
    enhanced = []
    for frame in (image, reference):
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        gray = cv2.createCLAHE(2., (12, 8)).apply(gray)
        gray = cv2.GaussianBlur(gray, (3, 3), 0)
        enhanced.append(gray)
        edges = cv2.Canny(gray, 25, 70)
        distances.append(cv2.distanceTransform(255-edges, cv2.DIST_L2, 5))
    detected = cv2.createLineSegmentDetector(cv2.LSD_REFINE_STD).detect(
        enhanced[0][y:y+height, x:x+width])[0]
    if detected is None:
        return [], []
    lines = detected[:, 0].reshape(-1, 2, 2) + [x, y]
    lengths = np.linalg.norm(lines[:, 1]-lines[:, 0], axis=1)
    lines = lines[lengths >= 22]
    stable_lines = []
    for line in lines:
        if min(edge_support(np.array([line[0], line[1]]), d)[0] for d in distances) < .80:
            continue
        xyz = plane_points(line, calibration, rim_m)
        vector = xyz[1]-xyz[0]
        length = np.linalg.norm(vector)
        if length < .020:
            continue
        stable_lines.append((line, vector/length, length))
    stable_lines.sort(key=lambda row: row[2], reverse=True)
    stable_lines = stable_lines[:90]
    candidates = []
    tried = set()
    families = set()
    for _, axis, _ in stable_lines[:45]:
        first = [(line, length) for line, vector, length in stable_lines
                 if abs(vector @ axis) > np.cos(np.deg2rad(6))]
        second = [(line, length) for line, vector, length in stable_lines
                  if abs(vector @ axis) < np.sin(np.deg2rad(6))]
        first = first[:16]; second = second[:16]
        family = (tuple(tuple(line.ravel()) for line, _ in first),
                  tuple(tuple(line.ravel()) for line, _ in second))
        if family in families:
            continue
        families.add(family)
        for (a, _), (b, _) in itertools.combinations(first, 2):
            for (c, _), (d, _) in itertools.combinations(second, 2):
                raw = [intersection(a, c), intersection(a, d), intersection(b, d), intersection(b, c)]
                if any(point is None for point in raw):
                    continue
                corners = np.array(raw)
                if (np.any(corners < [x+2, y+2]) or np.any(corners >= [x+width-2, y+height-2])
                        or not cv2.isContourConvex(corners.astype(np.float32))):
                    continue
                key = tuple(sorted(tuple(np.rint(p/3).astype(int)) for p in corners))
                if key in tried:
                    continue
                tried.add(key)
                xyz = plane_points(corners, calibration, rim_m)
                edges = np.roll(xyz, -1, axis=0)-xyz
                edge_lengths = np.linalg.norm(edges, axis=1)
                dimensions = np.sort([(edge_lengths[0]+edge_lengths[2])/2,
                                      (edge_lengths[1]+edge_lengths[3])/2])[::-1]
                if dimensions[0] < .100 or dimensions[1] < .050 or dimensions[0] > .5:
                    continue
                directions = edges/edge_lengths[:, None]
                if np.max(np.abs(np.sum(directions*np.roll(directions, -1, axis=0), axis=1))) > .10:
                    continue
                if max(abs(edge_lengths[0]-edge_lengths[2]), abs(edge_lengths[1]-edge_lengths[3])) > .01:
                    continue
                support = [edge_support(corners, distance) for distance in distances]
                if min(min(s) for s in support) < .60:
                    continue
                center = xyz.mean(0)
                candidates.append({'corners_px': corners.tolist(), 'corners_base_m': xyz.tolist(),
                    'center_at_rim_base_m': center.tolist(), 'dimensions_mm': (dimensions*1000).tolist(),
                    'edge_support_current_and_reference': support,
                    'score': float(sum(map(sum, support))),
                    'note': 'Geometric rectangle candidate only; inner rim and tool clearance are not yet verified'})
    candidates.sort(key=lambda row: row['score'], reverse=True)
    selected = []
    for candidate in candidates:
        center = np.array(candidate['center_at_rim_base_m'])
        if any(np.linalg.norm(center-np.array(row['center_at_rim_base_m'])) < .025 for row in selected):
            continue
        selected.append(candidate)
        if len(selected) >= 12:
            break
    return selected, [line.tolist() for line, _, _ in stable_lines]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--image', type=Path, required=True)
    parser.add_argument('--reference', type=Path, required=True)
    parser.add_argument('--session', type=Path, default=Path('captures/calibration/session-01'))
    parser.add_argument('--rim-mm', type=float, required=True)
    parser.add_argument('--roi', type=int, nargs=4, default=[0, 340, 650, 380])
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if not 0 < args.rim_mm <= 200:
        parser.error('Rim clearance must be 0..200 mm')
    image, reference = cv2.imread(str(args.image)), cv2.imread(str(args.reference))
    candidates, lines = rim_candidates(image, reference, load_plane(args.session), args.rim_mm/1000, args.roi)
    result = {'motion_sent': False, 'status': 'candidates_require_region_verification' if candidates else 'no_complete_supported_rim',
              'image': str(args.image.resolve()), 'reference': str(args.reference.resolve()),
              'rim_height_mm_user_provided': args.rim_mm, 'floor_height_mm_user_provided': 0,
              'method': 'RGB_LSD_four_static_edges_rectangle_on_calibrated_rim_plane',
              'roi_xywh': args.roi, 'candidates': candidates, 'static_lines_px': lines}
    args.output.mkdir(exist_ok=False, parents=True)
    (args.output/'rim-candidates.json').write_text(json.dumps(result, indent=2))
    annotated = image.copy()
    for line in lines:
        cv2.line(annotated, tuple(np.rint(line[0]).astype(int)), tuple(np.rint(line[1]).astype(int)), (0, 180, 255), 1)
    for index, row in enumerate(candidates):
        polygon = np.rint(row['corners_px']).astype(int)
        cv2.polylines(annotated, [polygon], True, (0, 255, 0), 2)
        center = np.rint(polygon.mean(0)).astype(int)
        cv2.putText(annotated, str(index+1), tuple(center), cv2.FONT_HERSHEY_SIMPLEX, .8, (0, 0, 255), 2)
    cv2.imwrite(str(args.output/'rim-candidates.jpg'), annotated)
    print(json.dumps({k: v for k, v in result.items() if k != 'static_lines_px'}, indent=2))


if __name__ == '__main__':
    main()
