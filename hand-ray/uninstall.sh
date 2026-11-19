#!/bin/sh
set -eu
DIR="$(cd "$(dirname "$0")" && pwd)/driver/frame_hand_ray"
VRPATHS="$HOME/.config/openvr/openvrpaths.vrpath"
[ -f "$VRPATHS" ] && cp "$VRPATHS" "$VRPATHS.bak-frame_hand_ray-uninstall-$(date +%Y%m%dT%H%M%S)"
/opt/steamvr/bin/linuxarm64/vrpathreg removedriver "$DIR"
echo "Unregistered. Restart the session (or reboot) to unload it."
