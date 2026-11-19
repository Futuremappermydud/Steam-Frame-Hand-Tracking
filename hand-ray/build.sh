#!/bin/sh
set -eu
HERE="$(cd "$(dirname "$0")" && pwd)"
HEADERS="${OPENVR_HEADERS:-/opt/steamvr/tools/hellovr_vulkan_linux/src/openvr/headers}"
if [ ! -f "$HEADERS/openvr_driver.h" ]; then
  echo "openvr_driver.h not in $HEADERS; using the vendored SDK 2.15.6 copy"
  HEADERS="$HERE/third_party/openvr"
fi
CXX="${CXX:-g++}"
OUT="$HERE/driver/frame_hand_ray/bin/linuxarm64"
mkdir -p "$OUT"
"$CXX" -std=c++17 -O2 -g -fPIC -shared -fvisibility=hidden -Wall -Wextra -Wshadow \
  -isystem "$HEADERS" "$HERE/src/driver_frame_hand_ray.cpp" -o "$OUT/driver_frame_hand_ray.so.new" -lpthread
mv -f "$OUT/driver_frame_hand_ray.so.new" "$OUT/driver_frame_hand_ray.so"
nm -D --defined-only "$OUT/driver_frame_hand_ray.so" | grep -q HmdDriverFactory || { echo "HmdDriverFactory not exported"; exit 1; }
sha256sum "$OUT/driver_frame_hand_ray.so"
