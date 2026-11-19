import json
import math
from pathlib import Path
import socket
import struct
import time
try:
    import numpy as np
except ImportError:  # pragma: no cover
    np = None

SOCKET_NAME = b'\0frame_hand_ray'
MAGIC, VERSION = b'FHR1', 3
HEADER = struct.Struct('<4sIqqI')
HAND = struct.Struct('<BBxxf3f3f')
XR_JOINTS = 26
SKELETON = struct.Struct('<B3x' + '7f' * XR_JOINTS)
QUALITY = struct.Struct('<6f')
WRIST, THUMB_TIP, INDEX_MCP, INDEX_TIP, MIDDLE_MCP = 0, 4, 5, 8, 9
PINCH_MIN_CONFIDENCE = 0.15
DEFAULT_DEVICE_CONFIG = {
    'cv': {'cad_from_cal': {'plus_x': [0.6369201046716755, -0.7704180733975734, 0.0280850929746303],
                            'plus_z': [-0.44326361175188, -0.3361624390475155, 0.8309706282837585],
                            'position': [-0.05284716302887107, -0.01865328053901913, 0.03697785218774215]}},
    'head': {'plus_x': [-1, 0, 0], 'plus_z': [0, 0, -1], 'position': [0, 0, 0]}}


def _config_frame(node):
    z = np.asarray(node['plus_z'], float); z /= np.linalg.norm(z)
    x = np.asarray(node['plus_x'], float); x -= x @ z * z; x /= np.linalg.norm(x)
    T = np.eye(4); T[:3, :3] = np.c_[x, np.cross(z, x), z]; T[:3, 3] = node['position']
    return T


def head_from_camera(device_config):
    data = json.loads(Path(device_config).read_text(encoding='utf-8')) if not isinstance(device_config, dict) else device_config
    return np.linalg.inv(_config_frame(data['head'])) @ _config_frame(data['cv']['cad_from_cal'])


def head_from_camera_or_default(device_config):
    try:
        return head_from_camera(device_config), str(device_config)
    except (OSError, ValueError, KeyError, TypeError):
        return head_from_camera(DEFAULT_DEVICE_CONFIG), 'built-in default (device config not readable)'


def monotonic_raw_ns():
    clock = getattr(time, 'CLOCK_MONOTONIC_RAW', None)
    return time.clock_gettime_ns(clock) if clock is not None else time.monotonic_ns()


class PinchDetector:
    def __init__(self, begin_m=.020, end_m=.055, end_frames=2, lost_s=.25, relative=True, drop_m=.020,
                 window_s=.6, close_max_m=.050, reopen_m=.015, release_floor_m=.035, hold_max_m=.030,
                 hold_max_s=.8, refractory_s=.15, begin_frames=2, min_confidence=0., unsure_hold_s=.5):
        if not 0 < begin_m < end_m:
            raise ValueError('Pinch thresholds need 0 < begin < end')
        self.begin_m, self.end_m, self.end_frames, self.lost_s = begin_m, end_m, int(end_frames), lost_s
        self.relative, self.drop_m, self.window_s, self.close_max_m = relative, drop_m, window_s, close_max_m
        self.reopen_m, self.release_floor_m, self.hold_max_m, self.hold_max_s = reopen_m, release_floor_m, hold_max_m, hold_max_s
        self.refractory_s, self.begin_frames = refractory_s, int(begin_frames)
        self.min_confidence, self.unsure_hold_s = min_confidence, unsure_hold_s
        self.down, self.open_frames, self.last_seen = False, 0, None
        self.recent, self.low, self.closing_frames, self.released, self.loose_since = [], 1., 0, -math.inf, None
        self.unsure_since, self.unsure_samples = None, 0

    def update(self, distance_m, t_s, confidence=None):
        if distance_m is None or not math.isfinite(distance_m):
            if self.last_seen is not None and t_s - self.last_seen > self.lost_s:
                self.recent.clear()
                if self.down:
                    self.down, self.open_frames, self.loose_since = False, 0, None
            return self.down
        if self.last_seen is not None and t_s - self.last_seen > self.lost_s:
            self.recent.clear()
            self.down, self.open_frames, self.loose_since = False, 0, None
        self.last_seen = t_s
        if confidence is not None and math.isfinite(confidence) and 0 <= confidence < self.min_confidence:
            self.unsure_samples += 1
            self.closing_frames = 0
            if self.unsure_since is None:
                self.unsure_since = t_s
            if self.down and t_s - self.unsure_since >= self.unsure_hold_s:
                self.down, self.open_frames, self.released, self.loose_since = False, 0, t_s, None
                self.recent.clear()
            return self.down
        self.unsure_since = None
        if not self.relative:
            return self._hysteresis(distance_m)
        self.recent.append((t_s, distance_m))
        while t_s - self.recent[0][0] > self.window_s:
            self.recent.pop(0)
        d = distance_m
        if not self.down:
            if d < self.begin_m:
                self._press(d)
            else:
                peak = max(x for _, x in self.recent)
                closing = d < self.close_max_m and peak - d >= self.drop_m and t_s - self.released >= self.refractory_s
                self.closing_frames = self.closing_frames + 1 if closing else 0
                if self.closing_frames >= self.begin_frames:
                    self._press(d)
        else:
            self.low = min(self.low, d)
            self.loose_since = (self.loose_since if self.loose_since is not None else t_s) if d > self.hold_max_m else None
            loose = self.loose_since is not None and t_s - self.loose_since >= self.hold_max_s
            if loose or d > self.end_m or (d - self.low >= self.reopen_m and d >= self.release_floor_m):
                self.open_frames += 1
                if self.open_frames >= self.end_frames or loose:
                    self.down, self.open_frames, self.released, self.loose_since = False, 0, t_s, None
                    self.recent = [(t_s, d)]
            else:
                self.open_frames = 0
        return self.down

    def _press(self, d):
        self.down, self.low, self.open_frames, self.closing_frames, self.loose_since = True, d, 0, 0, None

    def _hysteresis(self, d):
        if not self.down:
            if d < self.begin_m:
                self.down, self.open_frames = True, 0
        elif d > self.end_m:
            self.open_frames += 1
            if self.open_frames >= self.end_frames:
                self.down, self.open_frames = False, 0
        else:
            self.open_frames = 0
        return self.down


def _quality_values(hand):
    def value(x):
        return float(x) if x is not None and math.isfinite(x) else -1.0
    fingers = (hand or {}).get('finger_confidence') or []
    return [value((hand or {}).get('confidence'))] + [value(fingers[i]) if i < len(fingers) else -1.0 for i in range(5)]


def encode(capture_ns, sequence, hands, send_ns=None):
    out = [HEADER.pack(MAGIC, VERSION, int(capture_ns), int(send_ns if send_ns is not None else monotonic_raw_ns()),
                       sequence & 0xffffffff)]
    for h in hands:
        if h is None:
            out.append(HAND.pack(0, 0, -1.0, 0, 0, 0, 0, 0, 0))
        else:
            out.append(HAND.pack(1, int(bool(h['pinch'])), float(h['pinch_distance_m']), *map(float, h['anchor_head']),
                                 *map(float, h['wrist_head'])))
    for h in hands:
        joints = h.get('joints_head') if h is not None else None
        if joints is None:
            out.append(SKELETON.pack(0, *([0.0] * (7 * XR_JOINTS))))
        else:
            if len(joints) != XR_JOINTS:
                raise ValueError('joints_head needs 26 joints')
            out.append(SKELETON.pack(1, *(float(v) for joint in joints for v in joint)))
    for h in hands:
        out.append(QUALITY.pack(*_quality_values(h)))
    return b''.join(out)


def _quat_from_matrix(R):
    w = math.sqrt(max(0.0, 1 + R[0][0] + R[1][1] + R[2][2])) / 2
    x = math.copysign(math.sqrt(max(0.0, 1 + R[0][0] - R[1][1] - R[2][2])) / 2, R[2][1] - R[1][2])
    y = math.copysign(math.sqrt(max(0.0, 1 - R[0][0] + R[1][1] - R[2][2])) / 2, R[0][2] - R[2][0])
    z = math.copysign(math.sqrt(max(0.0, 1 - R[0][0] - R[1][1] + R[2][2])) / 2, R[1][0] - R[0][1])
    n = math.sqrt(x * x + y * y + z * z + w * w)
    return [x / n, y / n, z / n, w / n]


def _quat_multiply(a, b):
    ax, ay, az, aw = np.moveaxis(np.asarray(a, float), -1, 0)
    bx, by, bz, bw = np.moveaxis(np.asarray(b, float), -1, 0)
    return np.stack([aw * bx + ax * bw + ay * bz - az * by, aw * by - ax * bz + ay * bw + az * bx,
                     aw * bz + ax * by - ay * bx + az * bw, aw * bw - ax * bx - ay * by - az * bz], -1)


class HandRaySender:
    def __init__(self, head_from_camera_matrix, begin_m=.020, end_m=.055, socket_name=SOCKET_NAME,
                 min_confidence=0.):
        self.T = np.asarray(head_from_camera_matrix, float)
        self.q_T = np.asarray(_quat_from_matrix(self.T[:3, :3]))
        self.pinch = [PinchDetector(begin_m, end_m, min_confidence=min_confidence),
                      PinchDetector(begin_m, end_m, min_confidence=min_confidence)]
        self.socket_name = socket_name
        self.sock = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM) if hasattr(socket, 'AF_UNIX') else None
        if self.sock is not None:
            self.sock.setblocking(False)
        self.sequence = 0

    def _to_head(self, p):
        return self.T[:3, :3] @ np.asarray(p, float) + self.T[:3, 3]

    def process(self, record, t_s=None):
        t_s = time.monotonic() if t_s is None else t_s
        try:
            capture = int(record.get('source_timestamp_ns') or 0)
        except (TypeError, ValueError):
            capture = 0
        capture = capture or monotonic_raw_ns()
        by_side, skeleton, source = [None, None], [None, None], [None, None]
        for hand in record.get('hands', []):
            side = hand.get('hand_id')
            joints = hand.get('joints_m')
            if side not in (0, 1) or not joints or len(joints) != 21 or by_side[side] is not None:
                continue
            j = np.asarray(joints, float)
            if not np.isfinite(j).all():
                continue
            by_side[side] = j
            skeleton[side] = self._skeleton_head(hand)
            source[side] = hand
        payload = [None, None]
        for side in (0, 1):
            j = by_side[side]
            distance = float(np.linalg.norm(j[THUMB_TIP] - j[INDEX_TIP])) if j is not None else None
            down = self.pinch[side].update(distance, t_s, (source[side] or {}).get('pinch_confidence'))
            if j is None:
                continue
            payload[side] = {'pinch': down, 'pinch_distance_m': distance,
                             'anchor_head': self._to_head((j[INDEX_MCP] + j[MIDDLE_MCP]) / 2),
                             'wrist_head': self._to_head(j[WRIST]), 'joints_head': skeleton[side],
                             'confidence': source[side].get('confidence'),
                             'finger_confidence': source[side].get('finger_confidence')}
        self.sequence += 1
        packet = encode(capture, self.sequence, payload)
        if self.sock is not None:
            try:
                self.sock.sendto(packet, self.socket_name)
            except OSError:
                pass
        return packet

    def _skeleton_head(self, hand):
        points, quats = hand.get('landmarks_m'), hand.get('landmark_orientations_xyzw')
        if points is None or quats is None:
            return None
        points, quats = np.asarray(points, float), np.asarray(quats, float)
        if points.shape != (XR_JOINTS, 3) or quats.shape != (XR_JOINTS, 4) or \
                not (np.isfinite(points).all() and np.isfinite(quats).all()):
            return None
        norms = np.linalg.norm(quats, axis=1)
        if not (np.abs(norms - 1) < .05).all():
            return None
        head_points = points @ self.T[:3, :3].T + self.T[:3, 3]
        head_quats = _quat_multiply(self.q_T, quats / norms[:, None])
        return np.c_[head_points, head_quats]
