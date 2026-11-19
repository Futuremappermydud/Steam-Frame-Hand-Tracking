"""Check Mercury's two ONNX models in models/fp32/ against their SHA-256, and optionally write the fp16 variants
the launcher uses by default.

    python models/fetch_models.py            # check models/fp32/
    python models/fetch_models.py --fp16     # also fp16 -> models/fp16/ (needs requirements-tools.txt)

The fp32 models are stored in this repository (see models/README.md for their source and credit). One that is
missing or changed is downloaded again from Monado's hand-tracking-models repository at the pinned revision.
The fp16 files must come out byte-identical to the pins in python/frame_hand_tracking/mercury.py
(MODEL_VARIANTS), which the tracker enforces; requirements-tools.txt pins the converter versions that made them.
"""
import argparse
import hashlib
from pathlib import Path
import subprocess
import sys
import urllib.request

ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT.parent / 'python'))
from frame_hand_tracking.mercury import MODEL_REVISION, MODEL_VARIANTS  # noqa: E402

URL = 'https://gitlab.freedesktop.org/monado/utilities/hand-tracking-models/-/raw/{rev}/{name}'


def sha256(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--fp16', action='store_true', help='also convert to fp16 (models/fp16)')
    a = ap.parse_args()
    fp32 = ROOT / 'fp32'
    fp32.mkdir(exist_ok=True)
    for name, digest in MODEL_VARIANTS['fp32'].items():
        path = fp32 / name
        if not path.exists() or sha256(path) != digest:
            with urllib.request.urlopen(URL.format(rev=MODEL_REVISION, name=name), timeout=120) as r:
                path.write_bytes(r.read())
        if sha256(path) != digest:
            raise SystemExit(f'{name}: SHA-256 {sha256(path)} does not match the pin {digest}')
        print(f'fp32 {name} OK')
    if a.fp16:
        fp16 = ROOT / 'fp16'
        valid = all((fp16 / n).is_file() and sha256(fp16 / n) == d for n, d in MODEL_VARIANTS['fp16'].items())
        if not valid:
            subprocess.run([sys.executable, str(ROOT / 'convert_fp16.py'), str(fp32), str(fp16)], check=True)
        for name, digest in MODEL_VARIANTS['fp16'].items():
            got = sha256(fp16 / name)
            if got != digest:
                raise SystemExit(f'fp16 {name}: SHA-256 {got} does not match the pin {digest} '
                                 '(converter versions differ from requirements-tools.txt?)')
            print(f'fp16 {name} OK')


if __name__ == '__main__':
    main()
