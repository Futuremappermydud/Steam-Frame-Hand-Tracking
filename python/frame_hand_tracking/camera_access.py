import ctypes
import json
import math
import os
from pathlib import Path
import time

import numpy as np

ORDER = ('slam_right', 'slam_left', 'upper_left', 'upper_right')
SIZES = {name: ((1056, 1024) if name.startswith('slam') else (640, 480)) for name in ORDER}

FCA_OK, FCA_ERR_DISCONNECTED, FCA_ERR_NO_FRAME, FCA_ERR_TORN, FCA_ERR_TIMEOUT = 0, -7, -8, -9, -11
FCA_ERR_PENDING = -13
FCA_FRAME_FLAG_DARK = 1
TOO_DARK = 'too dark: the cameras see no bright frames'


class CameraInfo(ctypes.Structure):
    _fields_ = [('index', ctypes.c_uint32), ('width', ctypes.c_uint32), ('height', ctypes.c_uint32),
                ('stride', ctypes.c_uint32), ('pixel_format', ctypes.c_uint32),
                ('name', ctypes.c_char * 32), ('sensor', ctypes.c_char * 32)]


class FrameInfo(ctypes.Structure):
    _fields_ = [('set_index', ctypes.c_uint64), ('sof_ns', ctypes.c_int64 * 4), ('eof_ns', ctypes.c_int64 * 4),
                ('copy_end_ns', ctypes.c_int64), ('read_ns', ctypes.c_int64), ('mean_gray', ctypes.c_float),
                ('flags', ctypes.c_uint32)]


class Status(ctypes.Structure):
    _fields_ = [('state', ctypes.c_uint32), ('connected', ctypes.c_uint32), ('daemon_pid', ctypes.c_uint32),
                ('reserved', ctypes.c_uint32), ('heartbeat_age_ns', ctypes.c_int64),
                ('sets_published', ctypes.c_uint64), ('copy_races', ctypes.c_uint64),
                ('invalid_metadata', ctypes.c_uint64), ('dark_skipped', ctypes.c_uint64),
                ('source', ctypes.c_char * 32), ('status', ctypes.c_char * 128)]


assert ctypes.sizeof(CameraInfo) == 84 and ctypes.sizeof(FrameInfo) == 96 and ctypes.sizeof(Status) == 216


def default_library():
    if os.environ.get('FHL_FCA_LIBRARY'):
        return Path(os.environ['FHL_FCA_LIBRARY'])
    repository = Path(__file__).resolve().parents[3]
    return repository / 'Steam-Frame-Camera-Access/build/libframe_camera_access.so'


def load_library(path):
    lib = ctypes.CDLL(str(path))
    lib.fca_result_string.restype = ctypes.c_char_p
    lib.fca_result_string.argtypes = [ctypes.c_int32]
    lib.fca_now_ns.restype = ctypes.c_int64
    lib.fca_connect.argtypes = [ctypes.c_char_p, ctypes.POINTER(ctypes.c_void_p)]
    lib.fca_disconnect.argtypes = [ctypes.c_void_p]
    lib.fca_disconnect.restype = None
    lib.fca_camera_count.argtypes = [ctypes.c_void_p]
    lib.fca_get_camera_info.argtypes = [ctypes.c_void_p, ctypes.c_uint32, ctypes.POINTER(CameraInfo)]
    lib.fca_get_status.argtypes = [ctypes.c_void_p, ctypes.POINTER(Status)]
    lib.fca_wait_for_frame.argtypes = [ctypes.c_void_p, ctypes.c_uint64, ctypes.c_int32]
    lib.fca_read_latest.argtypes = [ctypes.c_void_p, ctypes.POINTER(ctypes.c_void_p), ctypes.POINTER(ctypes.c_uint64),
                                    ctypes.c_uint32, ctypes.POINTER(FrameInfo)]
    for name in ('fca_connect', 'fca_camera_count', 'fca_get_camera_info', 'fca_get_status', 'fca_wait_for_frame',
                 'fca_read_latest'):
        getattr(lib, name).restype = ctypes.c_int32
    return lib


class CameraAccessSource:
    def __init__(self, calibration, library=None, timeout=1, min_mean=10, max_age_ms=500., reconnect_s=1.,
                 max_reconnect_s=8., idle_wait_s=.1):
        if isinstance(max_age_ms, bool) or not isinstance(max_age_ms, (float, int)) or not math.isfinite(max_age_ms) \
                or not 0 < max_age_ms <= 2000:
            raise ValueError('Invalid source max age')
        self.library_path = Path(library) if library else default_library()
        if not self.library_path.is_file():
            raise RuntimeError(f'{self.library_path} not found: build Steam-Frame-Camera-Access (scripts/build.sh) '
                               'or set FHL_FCA_LIBRARY')
        self.lib = load_library(self.library_path)
        self.timeout, self.min_mean, self.max_age_ns = timeout, min_mean, int(max_age_ms * 1e6)
        self.reconnect_s, self.max_reconnect_s, self.idle_wait_s = reconnect_s, max_reconnect_s, idle_wait_s
        self.client = ctypes.c_void_p()
        self.next_connect = 0.
        self.retry_s = 0.
        self.last_set, self.last_sof = 0, 0
        self.last_wait_counts = None
        self.buffers, self.cameras = {}, {}
        self.state = 'starting'
        self.message = 'connecting to the camera access service'
        calibration_cameras = {c['id']: c for c in json.loads(Path(calibration).read_text())['cameras']}
        for name in ORDER:
            cal = calibration_cameras.get(name)
            if cal is None or (cal['width'], cal['height']) != SIZES[name]:
                raise RuntimeError(f'Calibration does not describe {name} at {SIZES[name]}')

    def connected(self):
        return bool(self.client.value)

    def _error(self, result):
        return self.lib.fca_result_string(result).decode()

    def _connect(self):
        now = time.monotonic()
        if now < self.next_connect:
            time.sleep(min(self.next_connect - now, self.idle_wait_s))
            raise TimeoutError(self.message)
        self.next_connect = now + self.reconnect_s
        client = ctypes.c_void_p()
        result = self.lib.fca_connect(None, ctypes.byref(client))
        if result != FCA_OK:
            asking = result == FCA_ERR_PENDING
            first_refusal = self.retry_s <= 0
            self.retry_s = self.reconnect_s if asking or first_refusal else min(self.retry_s * 2, self.max_reconnect_s)
            self.next_connect = now + self.retry_s
            self.state, self.message = 'waiting', self._error(result)
            raise TimeoutError(self.message)
        self.retry_s = 0.
        cameras = {}
        for i in range(self.lib.fca_camera_count(client)):
            info = CameraInfo()
            if self.lib.fca_get_camera_info(client, i, ctypes.byref(info)) == FCA_OK:
                cameras[info.name.decode()] = info
        if set(cameras) != set(ORDER) or any((cameras[n].width, cameras[n].height) != SIZES[n] for n in ORDER) \
                or any(cameras[n].stride != cameras[n].width for n in ORDER):
            self.lib.fca_disconnect(client)
            raise RuntimeError(f'Unexpected camera layout from the camera access service: {sorted(cameras)}')
        self.client, self.cameras = client, cameras
        self.buffers = {n: np.empty((cameras[n].height, cameras[n].width), np.uint8) for n in ORDER}
        self.last_set = 0
        self.state, self.message = 'waiting', 'connected; waiting for camera frames'

    def _disconnect(self, reason):
        if self.connected():
            self.lib.fca_disconnect(self.client)
        self.client = ctypes.c_void_p()
        self.state, self.message = 'waiting', reason
        self.next_connect = time.monotonic() + self.reconnect_s
        self.retry_s = 0.

    def read(self, selected_cameras):
        if not set(selected_cameras).issubset(ORDER):
            raise ValueError('Unknown tracking camera selection')
        if not self.connected():
            self._connect()
        deadline = time.monotonic() + self.timeout
        wanted = [n for n in ORDER if n in selected_cameras]
        dark_seen = 0
        while time.monotonic() < deadline:
            result = self.lib.fca_wait_for_frame(self.client, self.last_set, int(max(1, (deadline - time.monotonic()) * 1000)))
            if result == FCA_ERR_DISCONNECTED:
                self._disconnect('the camera access service went away; reconnecting')
                raise TimeoutError(self.message)
            if result == FCA_ERR_TIMEOUT:
                break
            if result != FCA_OK:
                raise RuntimeError(self._error(result))
            pointers = (ctypes.c_void_p * 4)(*[self.buffers[n].ctypes.data if n in wanted else None for n in ORDER])
            capacities = (ctypes.c_uint64 * 4)(*[self.buffers[n].nbytes for n in ORDER])
            info = FrameInfo()
            result = self.lib.fca_read_latest(self.client, pointers, capacities, 4, ctypes.byref(info))
            if result == FCA_ERR_DISCONNECTED:
                self._disconnect('the camera access service went away; reconnecting')
                raise TimeoutError(self.message)
            if result in (FCA_ERR_NO_FRAME, FCA_ERR_TORN):
                continue
            if result != FCA_OK:
                raise RuntimeError(self._error(result))
            self.last_set = info.set_index
            sof = info.sof_ns[0]
            if (info.flags & FCA_FRAME_FLAG_DARK) or info.mean_gray < self.min_mean or sof <= self.last_sof:
                dark_seen += sof > self.last_sof
                continue
            self.last_sof = sof
            self.last_wait_counts = None
            age = self.lib.fca_now_ns() - sof
            if not 0 <= age <= self.max_age_ns:
                raise TimeoutError('Live source exceeded configured age')
            self.state, self.message = 'streaming', 'streaming'
            return sof, {n: self.buffers[n] for n in wanted}
        status = Status()
        if self.connected() and self.lib.fca_get_status(self.client, ctypes.byref(status)) == FCA_OK:
            self.message = status.status.decode() or 'waiting for camera frames'
            counts = (status.sets_published, status.dark_skipped)
            previous = self.last_wait_counts
            if dark_seen or (previous is not None and counts[1] > previous[1] and counts[0] == previous[0]):
                self.message = TOO_DARK
            self.last_wait_counts = counts
        elif dark_seen:
            self.message = TOO_DARK
        self.state = 'waiting'
        raise TimeoutError(self.message)

    def close(self):
        if self.connected():
            self.lib.fca_disconnect(self.client)
        self.client = ctypes.c_void_p()
