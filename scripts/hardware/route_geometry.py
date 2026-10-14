"""Task-side Cartesian sampling and SDK-side joint route compaction."""

import math


def sample_tcp_path(poses, *, translation_step_m, rotation_step_deg):
    """System Python: sample [x,y,z,rx,ry,rz] TCP poses in metres/radians.

    Uses SDK RPY convention (extrinsic xyz). Include the live start pose first.
    Steps are explicit task choices, not clearance or collision guarantees.
    """
    import numpy as np
    from scipy.spatial.transform import Rotation, Slerp

    values = np.asarray(poses, dtype=float)
    if (values.ndim != 2 or values.shape[1] != 6 or len(values) < 2
            or not np.isfinite(values).all()
            or not math.isfinite(translation_step_m) or translation_step_m <= 0
            or not math.isfinite(rotation_step_deg) or rotation_step_deg <= 0):
        raise ValueError('Provide finite poses and positive sampling steps')
    result = []
    for start, end in zip(values[:-1], values[1:]):
        rotations = Rotation.from_euler('xyz', [start[3:], end[3:]])
        angle = (rotations[0].inv()*rotations[1]).magnitude()
        count = max(1, math.ceil(np.linalg.norm(end[:3]-start[:3])/translation_step_m),
                    math.ceil(math.degrees(angle)/rotation_step_deg))
        if count + len(result) > 10000:
            raise ValueError('Path exceeds 10000 samples')
        for t, rotation in zip(np.linspace(0, 1, count+1)[1:],
                               Slerp([0, 1], rotations)(np.linspace(0, 1, count+1)[1:])):
            matrix = np.eye(4)
            matrix[:3, :3] = rotation.as_matrix()
            matrix[:3, 3] = start[:3]+t*(end[:3]-start[:3])
            result.append(matrix.ravel().tolist())
    return result


def path_spec_from_poses(poses, tcp_bounds_m, rate=None, *, translation_step_m=.02,
                         rotation_step_deg=5., blend_radius_mm=1.):
    """Build a path spec retaining each supplied pose through IK compaction.

    20 mm / 5 degree sampling is a starting choice for clear paths, not a
    Cartesian error or collision bound. Use finer steps where required.
    """
    from scripts.hardware.control_settings import load_settings, validate_rate
    rate = validate_rate(load_settings()['robot']['sdk_rate'] if rate is None else rate)
    if len(poses) < 2:
        raise ValueError('Provide a start and at least one target pose')
    matrices, keys = [], []
    for first, last in zip(poses, poses[1:]):
        matrices.extend(sample_tcp_path([first, last], translation_step_m=translation_step_m,
                                        rotation_step_deg=rotation_step_deg))
        if len(matrices) > 10000:
            raise ValueError('Path exceeds 10000 samples')
        keys.append(len(matrices)-1)
    return {'tcp_matrices_row_major': matrices, 'keypoint_indices': keys,
            'tcp_bounds_m': tcp_bounds_m, 'rate': rate, 'blend_radius_mm': blend_radius_mm}


def compact_joints(start, samples, *, tolerance_deg=.01):
    """Keep joint-space corners and order; merge near-collinear samples.

    Cartesian fidelity still requires FK/bounds inspection after compaction.
    """
    points = [start] + samples
    result, first = [], 0
    while first < len(points)-1:
        best = first+1
        for last in range(first+1, len(points)):
            delta = [b-a for a, b in zip(points[first], points[last])]
            axis = max(range(6), key=lambda i: abs(delta[i]))
            if abs(delta[axis]) < 1e-9:
                break
            previous_t, fits = 0., True
            for row in points[first+1:last+1]:
                t = (row[axis]-points[first][axis])/delta[axis]
                if (not previous_t <= t <= 1.000000001
                        or max(abs(row[i]-points[first][i]-t*delta[i]) for i in range(6)) > tolerance_deg):
                    fits = False
                    break
                previous_t = t
            if not fits:
                break
            best = last
        point = points[best]
        if max(abs(a-b) for a, b in zip(points[first], point)) > 1e-9:
            result.append(point)
        first = best
    return result
