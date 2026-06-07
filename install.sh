#!/usr/bin/env bash
# Flow Calibration plugin installer.
#
# Symlinks the python modules into Klipper's extras dir and (optionally)
# registers the repo with Moonraker's update manager so Mainsail/Fluidd can
# update it. Re-running is safe (idempotent). Moonraker re-runs this on update.
#
# Overridable via env:
#   KLIPPER_PATH=~/klipper  MOONRAKER_CONF=~/printer_data/config/moonraker.conf
set -euo pipefail

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
KLIPPER_PATH="${KLIPPER_PATH:-${HOME}/klipper}"
EXTRAS_DIR="${KLIPPER_PATH}/klippy/extras"
MOONRAKER_CONF="${MOONRAKER_CONF:-${HOME}/printer_data/config/moonraker.conf}"
ALWAYS_MODULES=(flow_calibration.py)
# cs1237.py is only installed if the Klipper build doesn't already ship one.
# Some forks (the tunnelled Kobra S1 build) include their own cs1237.py that is
# shared with their bed probe — overwriting it breaks homing/probing. In that
# case flow_calibration.py talks to the existing module through an adapter.
OPTIONAL_SENSOR=cs1237.py

echo "=== Flow Calibration installer ==="
echo "  repo:     ${REPO_DIR}"
echo "  klipper:  ${KLIPPER_PATH}"

if [ ! -d "${EXTRAS_DIR}" ]; then
    echo "ERROR: Klipper extras dir not found: ${EXTRAS_DIR}"
    echo "       Set KLIPPER_PATH=/path/to/klipper and re-run."
    exit 1
fi

# 1) Symlink the always-installed modules (so 'git pull' updates live code).
for m in "${ALWAYS_MODULES[@]}"; do
    ln -sf "${REPO_DIR}/extras/${m}" "${EXTRAS_DIR}/${m}"
    echo "  linked ${m} -> ${EXTRAS_DIR}/${m}"
done

# 1b) Sensor module: never clobber a cs1237.py that the Klipper build ships
#     itself (a real file we didn't create). Install ours only when none
#     exists, or refresh our own symlink.
SENSOR_DEST="${EXTRAS_DIR}/${OPTIONAL_SENSOR}"
SENSOR_SRC="${REPO_DIR}/extras/${OPTIONAL_SENSOR}"
BUILD_HAS_OWN=0
if git -C "${KLIPPER_PATH}" cat-file -e "HEAD:klippy/extras/${OPTIONAL_SENSOR}" 2>/dev/null; then
    BUILD_HAS_OWN=1   # this Klipper build ships its own cs1237.py in git
fi

if [ "${BUILD_HAS_OWN}" = "1" ]; then
    # The Klipper build has its own cs1237.py (shared with its bed probe).
    # Never use ours — restore the build's version if a prior install replaced
    # it, then let flow_calibration.py adapt to it.
    if [ -L "${SENSOR_DEST}" ]; then
        rm -f "${SENSOR_DEST}"
        git -C "${KLIPPER_PATH}" checkout -- "klippy/extras/${OPTIONAL_SENSOR}" 2>/dev/null || true
        echo "  restored this Klipper build's own ${OPTIONAL_SENSOR} (probe-shared)"
    else
        echo "  keeping this Klipper build's own ${OPTIONAL_SENSOR} (probe-shared)"
    fi
    echo "    -> flow_calibration.py will use it via its adapter"
elif [ -L "${SENSOR_DEST}" ] || [ ! -e "${SENSOR_DEST}" ]; then
    # No build-shipped cs1237.py (e.g. mainline Klipper): install/refresh ours.
    ln -sf "${SENSOR_SRC}" "${SENSOR_DEST}"
    echo "  linked ${OPTIONAL_SENSOR} -> ${SENSOR_DEST}"
else
    # A real, untracked cs1237.py we didn't create — don't touch it.
    echo "  found existing ${OPTIONAL_SENSOR} (not git-tracked) — keeping it"
    echo "    -> flow_calibration.py will use it via its adapter"
fi

# 2) Register with Moonraker's update manager (once).
if [ -f "${MOONRAKER_CONF}" ]; then
    if ! grep -q "update_manager flow_calibration" "${MOONRAKER_CONF}"; then
        ORIGIN="$(git -C "${REPO_DIR}" remote get-url origin 2>/dev/null \
                  || echo 'https://github.com/CHANGE_ME/flow_calibration.git')"
        BRANCH="$(git -C "${REPO_DIR}" rev-parse --abbrev-ref HEAD 2>/dev/null \
                  || echo 'main')"
        cat >> "${MOONRAKER_CONF}" <<EOF

[update_manager flow_calibration]
type: git_repo
channel: stable
path: ${REPO_DIR}
origin: ${ORIGIN}
primary_branch: ${BRANCH}
install_script: install.sh
managed_services: klipper
EOF
        echo "  added [update_manager flow_calibration] to moonraker.conf"
        RESTART_MOONRAKER=1
    else
        echo "  moonraker update_manager block already present"
    fi
else
    echo "  NOTE: moonraker.conf not found at ${MOONRAKER_CONF}"
    echo "        add the [update_manager flow_calibration] block manually (see README.md)."
fi

# 3) Restart services.
if command -v systemctl >/dev/null 2>&1; then
    if [ "${RESTART_MOONRAKER:-0}" = "1" ]; then
        sudo systemctl restart moonraker || true
    fi
    sudo systemctl restart klipper || true
fi

echo "=== Done. Add [cs1237] + [flow_calibration] to printer.cfg (see README.md) ==="
