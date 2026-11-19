#!/bin/bash
set -euo pipefail
root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
python=${PYTHON:-python3.12}
command -v "$python" >/dev/null || python=python3
"$python" -c 'import sys; assert sys.version_info[:2] == (3, 12), sys.version' || {
  echo 'Python 3.12 is required (the pinned wheels are cp312 aarch64).' >&2; exit 2; }
test -x "$root/.venv/bin/python" || "$python" -m venv "$root/.venv"
lock=$root/requirements.lock
if [ "${1:-}" = --tools ]; then lock=$root/requirements-tools.lock; fi
"$root/.venv/bin/python" -m pip install --quiet --disable-pip-version-check --require-hashes --only-binary=:all: \
  -r "$lock"
"$root/.venv/bin/python" -c 'import numpy, cv2, onnxruntime; print("numpy", numpy.__version__, "| opencv", cv2.__version__, "| onnxruntime", onnxruntime.__version__)'
