#!/usr/bin/env python3
import os
from pathlib import Path
import sys

KEYS = ('shoulderWidth', 'shoulderDrop', 'shoulderBack', 'connectFrames', 'lostMs', 'staleMs', 'pinchHoldMs',
        'singleRay', 'filterMinCutoff', 'filterBeta', 'directionMinCutoff', 'directionBeta', 'predictGain',
        'predictLeadMs', 'predictMaxMs', 'velocitySmoothing', 'fingerFilter', 'fingerMinCutoff', 'fingerBeta',
        'fingerPredictMaxMs', 'confidenceLow', 'confidenceHigh', 'intentConeDeg', 'intentExitDeg', 'intentShowMs',
        'intentHideMs', 'intentGaze', 'handOutline', 'systemGesture', 'pointMinPalmDeg', 'pointMaxDownDeg',
        'pointHideMs', 'systemMaxViewDeg', 'systemEnterDeg', 'systemArmMs', 'systemHoldMs', 'pinchRewindMs',
        'pinchReleaseHoldMs', 'handSkeleton', 'gripBegin', 'gripEnd', 'pinchValueOpenM', 'pinchValueClosedM',
        'poseMode', 'handRotationMinCutoff', 'handRotationBeta', 'gazeMinCutoff', 'gazeBeta', 'gazeYawOffsetDeg',
        'gazePitchOffsetDeg', 'gazeBlinkOpenness', 'gazeStaleMs', 'controllerPolicy', 'controllerMoveSpeed',
        'controllerQuietMs', 'controllerHoldM')
path = Path(os.environ.get('XDG_RUNTIME_DIR') or f'/run/user/{os.getuid()}') / 'frame_hand_ray.tune'
values = {}
if path.exists():
    for line in path.read_text().splitlines():
        if '=' in line and not line.startswith('#'):
            key, value = line.split('=', 1)
            values[key] = value
args = sys.argv[1:]
if args == ['--reset']:
    values = {}
elif args and args != ['--show']:
    for item in args:
        key, value = item.split('=', 1)
        if key not in KEYS:
            raise SystemExit(f'Unknown key {key}; known: {", ".join(KEYS)}')
        float(value)
        values[key] = value
if args and args != ['--show']:
    temporary = path.with_name(path.name + '.tmp')
    temporary.write_text(''.join(f'{k}={v}\n' for k, v in values.items()))
    os.replace(temporary, path)
print(path, values or '(empty)')
