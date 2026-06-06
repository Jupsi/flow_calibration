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
MODULES=(cs1237.py flow_calibration.py)

echo "=== Flow Calibration installer ==="
echo "  repo:     ${REPO_DIR}"
echo "  klipper:  ${KLIPPER_PATH}"

if [ ! -d "${EXTRAS_DIR}" ]; then
    echo "ERROR: Klipper extras dir not found: ${EXTRAS_DIR}"
    echo "       Set KLIPPER_PATH=/path/to/klipper and re-run."
    exit 1
fi

# 1) Symlink the python modules (so 'git pull' updates the live code).
for m in "${MODULES[@]}"; do
    ln -sf "${REPO_DIR}/extras/${m}" "${EXTRAS_DIR}/${m}"
    echo "  linked ${m} -> ${EXTRAS_DIR}/${m}"
done

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
