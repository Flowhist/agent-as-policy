"""Locate a known rectangular top face, never a whole-object silhouette center."""

import argparse
import itertools
import json
from pathlib import Path

import cv2
import numpy as np
import pyrealsense2 as rs
import yaml



class TopFaceNotFound(ValueError):
    pass


class TopPlane:
    def __init__(self, camera, extrinsics, table, top_height_m):
        if top_height_m is not None and (not np.isfinite(top_height_m) or top_height_m <= 0):
            raise ValueError('Top height must be a positive measured value')
        self.top_height_m = top_height_m
        self.intr = rs.intrinsics()
        for key in ('width', 'height', 'fx', 'fy', 'ppx', 'ppy'):
            setattr(self.intr, key, camera[key])
        if camera['distortion_model'] != 'distortion.inverse_brown_conrady':
            raise ValueError('Unsupported camera distortion model')
        self.intr.model = rs.distortion.inverse_brown_conrady
        self.intr.coeffs = camera['coefficients']
        self.bc = np.array(extrinsics['T_base_camera'], dtype=float)
        self.normal = np.array(table['table_up_normal_base'], dtype=float)
        self.d = float(table['plane_d_m'])
        if (self.bc.shape != (4, 4) or not np.isfinite(self.bc).all()
                or not np.isclose(np.linalg.norm(self.normal), 1., atol=1e-6)):
            raise ValueError('Invalid calibration')

    def points(self, pixels):
        if self.top_height_m is None:
            raise ValueError('Top height is required for top-face projection')
        rays = np.array([rs.rs2_deproject_pixel_to_point(
            self.intr, list(map(float, p)), 1.) for p in pixels]) @ self.bc[:3, :3].T
        denominator = rays @ self.normal
        if np.any(np.abs(denominator) < 1e-6):
            raise ValueError('Ray parallel to top plane')
        distance = (self.top_height_m - self.d - self.normal @ self.bc[:3, 3]) / denominator
        if np.any(distance <= 0):
            raise ValueError('Top plane behind camera')
        return self.bc[:3, 3] + rays * distance[:, None]


def load_plane(session, top_height_m=None):
    return TopPlane(
        yaml.safe_load((session / 'samples.yaml').read_text())['camera'],
        yaml.safe_load((session / 'solve-15-validate-5/result.yaml').read_text()),
        yaml.safe_load((session / 'table-plane.yaml').read_text()), top_height_m)


def intersection(a, b):
    p, q = np.asarray(a).reshape(2, 2), np.asarray(b).reshape(2, 2)
    matrix = np.column_stack([p[1] - p[0], -(q[1] - q[0])])
    if abs(np.linalg.det(matrix)) < 1e-6:
        return None
    t = np.linalg.solve(matrix, q[0] - p[0])[0]
    return p[0] + t * (p[1] - p[0])


def detect_top(image, plane, size_m, roi=None):
    size_m = np.sort(np.asarray(size_m, dtype=float))[::-1]
    if size_m.shape != (2,) or not np.isfinite(size_m).all() or np.any(size_m <= 0):
        raise ValueError('Top dimensions must contain two positive measured lengths')
    if image is None or image.shape[:2] != (plane.intr.height, plane.intr.width):
        raise TopFaceNotFound('Image dimensions do not match calibrated intrinsics')
    height, width = image.shape[:2]
    x, y, w, h = roi or (0, 0, width, height)
    if min(x, y) < 0 or min(w, h) <= 0 or x+w > width or y+h > height:
        raise ValueError('Invalid image ROI')
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    red = cv2.inRange(hsv, (0, 100, 70), (12, 255, 255)) | cv2.inRange(hsv, (170, 100, 70), (179, 255, 255))
    region = np.zeros_like(red)
    region[y:y+h, x:x+w] = red[y:y+h, x:x+w]
    contours, _ = cv2.findContours(region, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    contours = [c for c in contours if cv2.contourArea(c) >= 400]
    if not contours:
        raise TopFaceNotFound('No sufficiently visible red target')
    candidates = []
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    for contour in contours:
        bx, by, bw, bh = cv2.boundingRect(contour)
        xa, ya = max(x, bx-3), max(y, by-3)
        xb, yb = min(x+w, bx+bw+3), min(y+h, by+bh+3)
        crop = gray[ya:yb, xa:xb]
        detected = cv2.createLineSegmentDetector(cv2.LSD_REFINE_STD).detect(crop)[0]
        if detected is None:
            continue
        lines = detected[:, 0].reshape(-1, 2, 2) + [xa, ya]
        lengths = np.linalg.norm(lines[:, 1]-lines[:, 0], axis=1)
        lines = lines[lengths >= 8]
        box = cv2.boxPoints(cv2.minAreaRect(contour))
        direction = box[1]-box[0]
        direction /= np.linalg.norm(direction)
        groups = [[], []]
        for line in lines:
            vector = line[1]-line[0]
            length = np.linalg.norm(vector)
            parallel = abs(vector @ direction / length)
            group = 0 if parallel > np.cos(np.deg2rad(15)) else 1 if parallel < np.sin(np.deg2rad(15)) else None
            if group is not None:
                groups[group].append((length, line))
        # Strong, extended boundaries constrain the top; color alone cannot do so.
        groups = [[p[1] for p in sorted(g, key=lambda p: p[0], reverse=True)[:14]] for g in groups]
        edges = cv2.Canny(crop, 20, 60)
        distance = cv2.distanceTransform(255-edges, cv2.DIST_L2, 5)
        for a, b in itertools.combinations(groups[0], 2):
            for c, e in itertools.combinations(groups[1], 2):
                corners = [intersection(a, c), intersection(a, e), intersection(b, e), intersection(b, c)]
                if any(p is None for p in corners):
                    continue
                corners = np.array(corners)
                if (np.any(corners < [xa, ya]) or np.any(corners >= [xb-1, yb-1])
                        or not cv2.isContourConvex(corners.astype(np.float32))):
                    continue
                xyz = plane.points(corners)
                vectors = np.roll(xyz, -1, axis=0)-xyz
                lengths = np.linalg.norm(vectors, axis=1)
                if min(lengths) < .005:
                    continue
                dimensions = np.sort([(lengths[0]+lengths[2])/2, (lengths[1]+lengths[3])/2])[::-1]
                relative_error = np.abs(dimensions-size_m)/size_m
                if np.max(relative_error) > .08:
                    continue
                units = vectors/lengths[:, None]
                if np.max(np.abs(np.sum(units*np.roll(units, -1, axis=0), axis=1))) > .12:
                    continue
                if max(abs(lengths[0]-lengths[2]), abs(lengths[1]-lengths[3])) > .004:
                    continue
                support = []
                for first, last in zip(corners, np.roll(corners, -1, axis=0)):
                    samples = first + np.linspace(.08, .92, 40)[:, None]*(last-first)
                    local = (samples-[xa, ya]).astype(np.float32)
                    distances = cv2.remap(distance, local[:, 0:1], local[:, 1:2], cv2.INTER_LINEAR).ravel()
                    support.append(float(np.mean(distances <= 1.1)))
                if min(support) < .7:
                    continue
                # At an oblique view the near top edge borders a red side face,
                # not the white background at the bottom of the whole silhouette.
                center = xyz.mean(axis=0)
                toward_camera = plane.bc[:3, 3]-center
                height_to_camera = toward_camera @ plane.normal
                tangent = toward_camera-height_to_camera*plane.normal
                side_red_fraction = None
                if np.linalg.norm(tangent) > .2*abs(height_to_camera):
                    middles = (xyz+np.roll(xyz, -1, axis=0))/2
                    outward = middles-center
                    outward /= np.linalg.norm(outward, axis=1)[:, None]
                    near = int(np.argmax(outward @ tangent))
                    first, last = corners[near], corners[(near+1)%4]
                    direction_out = (first+last)/2-corners.mean(axis=0)
                    direction_out /= np.linalg.norm(direction_out)
                    samples = first+np.linspace(.15, .85, 30)[:, None]*(last-first)+2*direction_out
                    pixels = np.rint(samples).astype(int)
                    if np.any(pixels < 0) or np.any(pixels >= [width, height]):
                        continue
                    side_red_fraction = float(np.mean(red[pixels[:, 1], pixels[:, 0]] > 0))
                    if side_red_fraction < .65:
                        continue
                top_mask = np.zeros_like(red)
                cv2.fillConvexPoly(top_mask, np.rint(corners).astype(np.int32), 255)
                interior = cv2.erode(top_mask, np.ones((3, 3), np.uint8)) > 0
                visibility = float(np.mean(red[interior] > 0)) if np.any(interior) else 0.
                if visibility < .94:
                    continue
                score = float(np.mean(support) + visibility - 2*np.mean(relative_error))
                long = units[np.argmax(lengths)]
                candidates.append({'score': score, 'center_base_m': xyz.mean(axis=0).tolist(),
                    'corners_px': corners.tolist(), 'corners_base_m': xyz.tolist(),
                    'long_axis_base': long.tolist(), 'size_measured_mm': (dimensions*1000).tolist(),
                    'edge_support': support, 'top_red_fraction': visibility,
                    'near_edge_exterior_side_red_fraction': side_red_fraction})
    if not candidates:
        raise TopFaceNotFound('No verified top face: edges, dimensions or visibility insufficient; no silhouette fallback')
    candidates.sort(key=lambda c: c['score'], reverse=True)
    best = candidates[0]
    for other in candidates[1:]:
        separation = np.linalg.norm(np.array(other['center_base_m'])-best['center_base_m'])
        if best['score']-other['score'] < .08 and separation > .0025:
            raise TopFaceNotFound('Ambiguous top-face hypotheses; no center emitted')
    return {'status': 'top_face_detected_not_motion_validated', 'method': 'four_observed_edges_on_known_top_plane',
            'top_size_mm': (size_m*1000).tolist(), 'top_height_mm': plane.top_height_m*1000,
            'roi_xywh': [x, y, w, h], **best,
            'limitations': ['Provisional table plane and calibration error remain',
                           'Requires four supported edges and substantially unobstructed top',
                           'Image edge thresholds are not a physical accuracy guarantee']}


def save_result(image, result, output):
    (output / 'top-face.json').write_text(json.dumps(result, indent=2))
    overlay = image.copy()
    mask = np.zeros(image.shape[:2], dtype=np.uint8)
    if 'corners_px' in result:
        corners = np.rint(result['corners_px']).astype(np.int32)
        cv2.fillConvexPoly(mask, corners, 255)
        cv2.polylines(overlay, [corners], True, (0, 255, 0), 2)
    else:
        cv2.putText(overlay, 'TOP FACE REJECTED - NO TARGET', (20, 35), cv2.FONT_HERSHEY_SIMPLEX, .7, (0, 0, 255), 2)
    cv2.imwrite(str(output / 'top-mask.png'), mask)
    cv2.imwrite(str(output / 'top-overlay.jpg'), overlay)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--image', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--session', type=Path, required=True)
    parser.add_argument('--size-mm', type=float, nargs=2, required=True)
    parser.add_argument('--top-height-mm', type=float, required=True)
    parser.add_argument('--roi', type=int, nargs=4, required=True)
    args = parser.parse_args()
    plane = load_plane(args.session, args.top_height_mm/1000)
    image = cv2.imread(str(args.image))
    if image is None:
        parser.error('Image could not be read')
    args.output.mkdir(parents=True, exist_ok=False)
    try:
        result = detect_top(image, plane, np.asarray(args.size_mm)/1000, args.roi)
    except TopFaceNotFound as exc:
        result = {'status': 'rejected', 'reason': str(exc)}
    result.update(image=str(args.image.resolve()), session=str(args.session.resolve()),
                  requested_size_mm=args.size_mm, requested_top_height_mm=args.top_height_mm,
                  motion_sent=False)
    save_result(image, result, args.output)
    print(json.dumps(result, indent=2))
    return int(result['status'] == 'rejected')


if __name__ == '__main__':
    raise SystemExit(main())
