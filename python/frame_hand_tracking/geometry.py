import json
from pathlib import Path
import numpy as np

TRACKING = ('slam_right', 'slam_left', 'upper_left', 'upper_right')


def validate_transform(value):
    matrix = np.asarray(value, dtype=float)
    if matrix.shape != (4, 4) or not np.isfinite(matrix).all() or not np.allclose(
            matrix[3], [0, 0, 0, 1], atol=1e-10, rtol=0):
        raise ValueError('Invalid finite homogeneous camera transform')
    if not np.allclose(matrix[:3, :3].T @ matrix[:3, :3], np.eye(3), atol=1e-4, rtol=0) \
            or abs(np.linalg.det(matrix[:3, :3]) - 1) > 1e-4:
        raise ValueError('Camera transform must be right-handed and approximately rigid')
    return matrix


def load_rig(path):
    """The four tracking cameras of a calibration file, by id, with their transforms checked."""
    rows = [c for c in json.loads(Path(path).read_text(encoding='utf-8-sig'))['cameras'] if c['id'] in TRACKING]
    cameras = {c['id']: dict(c) for c in rows}
    if set(cameras) != set(TRACKING) or len(rows) != 4:
        raise ValueError('The calibration needs four uniquely identified tracking cameras')
    for camera in cameras.values():
        validate_transform(camera['T_reference_from_camera_candidate'])
    return {name: cameras[name] for name in TRACKING}
