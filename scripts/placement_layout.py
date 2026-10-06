"""Offline placement-clearance study from a visually identified RGB tray rim."""
import argparse
import json
from pathlib import Path

import cv2
import numpy as np
import pyrealsense2 as rs

from block_top import load_plane
from placement_rgb import plane_points


def project(points, calibration):
    camera = (np.asarray(points) - calibration.bc[:3, 3]) @ calibration.bc[:3, :3]
    if np.any(camera[:, 2] <= 0):
        raise ValueError('Point behind camera')
    normalized = camera[:, :2] / camera[:, 2:3]
    scale = np.array([calibration.intr.fx, calibration.intr.fy])
    pixels = normalized * scale + [calibration.intr.ppx, calibration.intr.ppy]
    for _ in range(12):
        rays = np.array([rs.rs2_deproject_pixel_to_point(calibration.intr, p.tolist(), 1.)
                         for p in pixels])
        pixels += (normalized - rays[:, :2] / rays[:, 2:3]) * scale
    return pixels


def rectangle(center, axes, size):
    return np.array(center) + np.array([[-1, -1], [1, -1], [1, 1], [-1, 1]]) * (np.array(size)/2) @ axes


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--rim', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    data = json.loads(args.rim.read_text())
    row = data['candidates'][0]
    cal = load_plane(Path('captures/calibration/session-01'))
    image = cv2.imread(data['image'])
    rim = np.array(row['corners_base_m'])[:, :2]
    long_axis = rim[0] - rim[1]
    long_axis /= np.linalg.norm(long_axis)
    inward = np.array([-long_axis[1], long_axis[0]])
    axes = np.array([long_axis, inward])
    origin = rim[1]
    local_rim = (rim-origin) @ axes.T
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, (0, 85, 35), (179, 255, 255))
    roi = np.zeros(mask.shape, np.uint8)
    cv2.fillConvexPoly(roi, np.rint(row['corners_px']).astype(np.int32), 255)
    mask &= cv2.erode(roi, np.ones((7, 7), np.uint8))
    count, labels, stats, _ = cv2.connectedComponentsWithStats(mask)
    mask = np.isin(labels, [i for i in range(1, count) if stats[i, cv2.CC_STAT_AREA] >= 12]).astype(np.uint8)*255
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    obstacles = []
    for contour in contours:
        pixels = contour[:, 0, :]
        # Existing pieces have unknown top heights. Sweep the full floor-to-rim interval.
        xy = np.concatenate([plane_points(pixels, cal, h)[:, :2] for h in (0., .05)])
        obstacles.append(cv2.convexHull(((xy-origin) @ axes.T).astype(np.float32)))
    polygon = local_rim.astype(np.float32)
    options = []
    sizes = [(.09023, .02986), (.11, .05), (.11, .06), (.11, .07), (.11, .08), (.13, .10)]
    for length, width in sizes:
        best = None
        for u in np.arange(.06, .22, .002):
            for v in np.arange(.025, .14, .002):
                center = np.array([u, v])
                corners = rectangle(center, np.eye(2), [length, width])
                edge_gap = min(cv2.pointPolygonTest(polygon, tuple(map(float, p)), True) for p in corners)
                if edge_gap < .012:
                    continue
                # Rectangular Minkowski expansion of each occupancy hull.
                gaps = []
                for obstacle in obstacles:
                    expanded = cv2.convexHull(np.concatenate([obstacle[:, 0, :] + p for p in corners-center]).astype(np.float32))
                    gaps.append(-cv2.pointPolygonTest(expanded, tuple(map(float, center)), True))
                obstacle_gap = min(gaps, default=1.)
                if obstacle_gap < .01:
                    continue
                score = min(edge_gap-.005, obstacle_gap)
                if best is None or score > best['score_m']:
                    best = {'center_local_m': center.tolist(), 'center_base_xy_m': (origin+center@axes).tolist(),
                            'size_m': [length, width], 'rim_gap_m': edge_gap,
                            'obstacle_gap_m': obstacle_gap, 'score_m': score}
        options.append({'size_m': [length, width], 'best': best})
    def on_plane(xy, height):
        xy = np.asarray(xy)
        return np.column_stack([xy, (height-cal.d-xy@cal.normal[:2])/cal.normal[2]])
    annotated = image.copy()
    cv2.polylines(annotated, [np.rint(row['corners_px']).astype(np.int32)], True, (0, 255, 0), 2)
    for obstacle in obstacles:
        pixels = project(on_plane(origin+obstacle[:, 0, :]@axes, 0.), cal)
        cv2.polylines(annotated, [np.rint(pixels).astype(np.int32)], True, (0, 0, 255), 1)
    chosen = next((item['best'] for item in reversed(options) if item['best']), None)
    if chosen:
        for size, color in [([.09023, .02986], (255, 255, 0)), (chosen['size_m'], (255, 0, 255))]:
            corners = rectangle(chosen['center_base_xy_m'], axes, size)
            pixels = project(on_plane(corners, 0.), cal)
            cv2.polylines(annotated, [np.rint(pixels).astype(np.int32)], True, color, 2)
    result = {'motion_sent': False, 'status': 'layout_study_not_execution_clearance',
              'source': str(args.rim.resolve()), 'rim_base_xy_m': rim.tolist(),
              'axes_long_inward_xy': axes.tolist(), 'origin_base_xy_m': origin.tolist(),
              'occupied_hulls_local_m': [p[:, 0, :].tolist() for p in obstacles],
              'height_sweep_m': [0, .05], 'options': options,
              'yaw_rad': float(np.arctan2(inward[1], inward[0])-np.pi),
              'limitations': ['Carrier sizes are sensitivity studies, not measured gripper/cable envelopes.',
                              'Rim is visually identified; occupancy requires visual review.']}
    args.output.mkdir(parents=True, exist_ok=False)
    (args.output/'layout.json').write_text(json.dumps(result, indent=2))
    cv2.imwrite(str(args.output/'layout.jpg'), annotated)
    cv2.imwrite(str(args.output/'occupancy-mask.png'), mask)
    print(json.dumps({k: result[k] for k in ('options', 'yaw_rad', 'limitations')}, indent=2))


if __name__ == '__main__':
    main()
