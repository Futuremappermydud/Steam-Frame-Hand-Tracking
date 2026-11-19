#!/usr/bin/env bash
# Removes Frame Hand Tracking from SteamVR: the dashboard panel, the hand tracking service and the driver
# registration. Run it as your user, or with sudo.
#   bash uninstall.sh            hand tracking only
#   sudo bash uninstall.sh --all also the Frame Camera Access service (other apps may be using it)
# The two source folders stay; delete them afterwards if you want.
set -euo pipefail
root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
camera=${FCA_SOURCE_DIR:-$(dirname -- "$root")/Steam-Frame-Camera-Access}
all=0
case ${1:-} in
  '') ;;
  --all) all=1 ;;
  -h | --help) sed -n '2,6s/^# \{0,1\}//p' "${BASH_SOURCE[0]}"; exit 0 ;;
  *) echo "Unknown argument: $1 (see --help)" >&2; exit 2 ;;
esac

as_user=()
if [[ $EUID -eq 0 ]]; then
  uid=${SUDO_UID:-}
  [[ -n $uid && $uid != 0 ]] || { echo "run it as your user, or through sudo from your own account" >&2; exit 1; }
  as_user=(sudo -u "$(id -nu "$uid")" env HOME="$(getent passwd "$uid" | cut -d: -f6)"
    XDG_RUNTIME_DIR="/run/user/$uid" DBUS_SESSION_BUS_ADDRESS="unix:path=/run/user/$uid/bus")
elif ((all)); then
  echo "--all removes the camera service, which needs sudo: sudo bash $root/uninstall.sh --all" >&2; exit 1
fi

"${as_user[@]}" bash "$root/scripts/install-panel.sh" --uninstall
"${as_user[@]}" sh "$root/hand-ray/uninstall.sh" || echo "the driver was not registered"
if ((all)); then
  bash "$camera/scripts/uninstall-daemon.sh"
fi
echo "Removed. Restart the headset to unload the driver. The sources are still in $root."
