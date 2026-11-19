import hashlib
import json
import math
from pathlib import Path
import sys

TRACKING = ('slam_right', 'slam_left', 'upper_left', 'upper_right')


def cross(a, b):
    return [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]]


def camera_row(c):
    kb = [i for i in c['intrinsics'] if i.get('cameraModel') == 'kb']
    if len(kb) != 1:
        raise ValueError(f"{c['sourceCamera']}: expected one 'kb' intrinsic model")
    kb = kb[0]
    e = c['extrinsics']
    if e.get('__units') != 'mm':
        raise ValueError(f"{c['sourceCamera']}: extrinsics must be in mm")
    x, z = [float(v) for v in e['plus_x']], [float(v) for v in e['plus_z']]
    y = cross(z, x)
    if abs(sum(a * b for a, b in zip(x, z))) > 1e-4 or abs(math.dist(x, [0, 0, 0]) - 1) > 1e-4 or \
            abs(math.dist(z, [0, 0, 0]) - 1) > 1e-4:
        raise ValueError(f"{c['sourceCamera']}: plus_x / plus_z are not orthonormal")
    t = [float(v) * 0.001 for v in e['position']]
    T = [[x[0], y[0], z[0], t[0]], [x[1], y[1], z[1], t[1]], [x[2], y[2], z[2], t[2]], [0.0, 0.0, 0.0, 1.0]]
    K = [[kb['fx'], 0, kb['cx']], [0, kb['fy'], kb['cy']], [0, 0, 1]]
    kb4 = [kb['k1'], kb['k2'], kb['k3'], kb['k4']]
    return {'id': c['sourceCamera'], 'name': c['sourceCamera'], 'width': int(c['width']), 'height': int(c['height']),
            'model': 'KB4_factory', 'K': K, 'factory_K': K, 'distortion': kb4, 'factory_kb4': kb4,
            'calibrated_image_circle_diameter_degrees': c['imageCircleDiameterInDeg'],
            'factory_parent': c.get('parent'), 'T_reference_from_camera_candidate': T}


def main():
    source = Path(sys.argv[1] if len(sys.argv) > 1 else '/persist/xrservice.json')
    out = Path(sys.argv[2]) if len(sys.argv) > 2 else Path(__file__).resolve().parent / 'frame-camera-rig.json'
    data = source.read_bytes()
    cameras = {c['sourceCamera']: c for c in json.loads(data)['cameras']}
    missing = [c for c in TRACKING if c not in cameras]
    if missing:
        raise SystemExit(f'{source}: tracking cameras missing: {missing}')
    rows = [camera_row(cameras[c]) for c in TRACKING]
    if rows[0]['T_reference_from_camera_candidate'] != [[1.0, 0.0, 0.0, 0.0], [0.0, 1.0, 0.0, 0.0],
                                                         [0.0, 0.0, 1.0, 0.0], [0.0, 0.0, 0.0, 1.0]]:
        raise SystemExit('slam_right is not the factory reference (identity); this exporter does not rebase')
    rig = {'schema_version': 1, 'coordinate_frame': 'slam_right_optical_metres',
           'reference': 'SLAM-right optical frame (factory camera0)',
           'source_pins': {'xrservice_json_sha256': hashlib.sha256(data).hexdigest(),
                           'exporter': 'calibration/export_rig.py'},
           'cameras': rows}
    out.write_text(json.dumps(rig, indent=2) + '\n', encoding='utf-8')
    print(f'wrote {out} from {source}')


if __name__ == '__main__':
    main()
