import numpy as np

METHODS = ('stock', 'mercury_rot')
DETECTOR_CROP = 0.7


def controls(value):
    method = value.get('acquisition_method', 'stock')
    if method not in METHODS:
        raise ValueError('Invalid acquisition method')
    roll = value.get('mercury_crop_roll_degrees', 0)
    if type(roll) is not int or roll not in (0, 90, 180, 270):
        raise ValueError('Invalid Mercury crop roll')
    return {'method': method, 'crop_roll_degrees': roll}


def detector_rotation_degrees(cameras):
    """Per camera of the rig, the image rotation (degrees) that puts the headset's up direction at the top."""
    transforms = {name: np.asarray(camera['T_reference_from_camera_candidate'], dtype=np.float64)
                  for name, camera in cameras.items()}
    left, right = transforms['slam_left'], transforms['slam_right']
    forward = left[:3, 2] + right[:3, 2]
    forward /= np.linalg.norm(forward)
    side = (left[:3, 3] - right[:3, 3]) - np.dot(left[:3, 3] - right[:3, 3], forward) * forward
    side /= np.linalg.norm(side)
    up = np.cross(forward, side)
    up /= np.linalg.norm(up)
    out = {}
    for name, transform in transforms.items():
        u = transform[:3, :3].T @ up
        out[name] = float((np.degrees(np.arctan2(u[1], u[0])) + 90) % 360)
    return out
