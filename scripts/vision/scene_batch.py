"""Run independent task-owned analyzers on one decoded frame in system Python."""

from concurrent.futures import ThreadPoolExecutor
import hashlib
from pathlib import Path
import time

import cv2
import numpy as np


def analyze_frame(image_path, jobs, *, workers=2):
    """Return per-job results/errors and timings; no capture, SDK or model calls.

    jobs maps names to callables(image_bgr). Jobs share a read-only image and
    must not mutate shared calibration or submit dependent work to this pool.
    Copy the image for overlays. Load calibration once in the caller, before
    submitting independent jobs. A failed job stays failed; no implicit retry.
    """
    if (not isinstance(jobs, dict) or not jobs
            or any(not isinstance(name, str) or not name or not callable(job)
                   for name, job in jobs.items())):
        raise ValueError('Provide named analyzer callables')
    if type(workers) is not int or workers < 1:
        raise ValueError('workers must be a positive integer')
    started = time.monotonic()
    path = Path(image_path).resolve()
    encoded = path.read_bytes()
    image = cv2.imdecode(np.frombuffer(encoded, dtype=np.uint8), cv2.IMREAD_COLOR)
    if image is None:
        raise ValueError(f'Cannot decode frame: {path}')
    image.setflags(write=False)

    def run(job):
        begin = time.monotonic()
        try:
            value = {'status': 'ok', 'result': job(image)}
        except Exception as exc:
            value = {'status': 'error', 'error': f'{type(exc).__name__}: {exc}'}
        return dict(value, elapsed_s=time.monotonic()-begin)

    with ThreadPoolExecutor(max_workers=min(workers, len(jobs))) as pool:
        futures = {name: pool.submit(run, job) for name, job in jobs.items()}
        results = {name: future.result() for name, future in futures.items()}
    return {'image_path': str(path), 'image_sha256': hashlib.sha256(encoded).hexdigest(),
            'status': 'ok' if all(r['status'] == 'ok' for r in results.values()) else 'incomplete',
            'jobs': results, 'elapsed_s': time.monotonic()-started}
