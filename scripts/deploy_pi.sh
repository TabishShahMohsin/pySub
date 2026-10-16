#!/usr/bin/env bash
# Deploy the ROV-side code to the Raspberry Pi over SSH.
#
# Usage: scripts/deploy_pi.sh [--install-deps] [--restart]
#
#   --install-deps  create/update the virtualenv and install requirements-pi.txt
#   --restart       restart the rov-pi service after deploying (arms the vehicle!)
#
# Environment:
#   ROV_PI_HOST  Pi hostname or IP   (default: auv.local)
#   ROV_PI_USER  SSH user on the Pi  (default: pi)
#   ROV_PI_DIR   install directory   (default: /home/$ROV_PI_USER/pysub)
# Variables are expanded locally on purpose before being sent over SSH.
# shellcheck disable=SC2029
set -euo pipefail

PI_HOST="${ROV_PI_HOST:-auv.local}"
PI_USER="${ROV_PI_USER:-pi}"
PI_DIR="${ROV_PI_DIR:-/home/${PI_USER}/pysub}"
TARGET="${PI_USER}@${PI_HOST}"

install_deps=false
restart=false
for arg in "$@"; do
    case "$arg" in
        --install-deps) install_deps=true ;;
        --restart) restart=true ;;
        -h | --help)
            sed -n '2,12p' "$0"
            exit 0
            ;;
        *)
            echo "Unknown option: $arg" >&2
            exit 2
            ;;
    esac
done

cd "$(dirname "$0")/.."

echo ">> Syncing code to ${TARGET}:${PI_DIR}"
ssh "$TARGET" "mkdir -p '${PI_DIR}'"
rsync -az --delete \
    --exclude '__pycache__' --exclude '*.pyc' \
    pi hardware_tests deploy requirements-pi.txt \
    "${TARGET}:${PI_DIR}/"

if [ "$install_deps" = true ]; then
    echo ">> Installing dependencies"
    # --system-site-packages keeps apt-installed packages (picamera2, librealsense) visible.
    ssh "$TARGET" "cd '${PI_DIR}' \
        && python3 -m venv --system-site-packages .venv \
        && .venv/bin/pip install --upgrade pip \
        && .venv/bin/pip install -r requirements-pi.txt"
fi

echo ">> Installing systemd unit (not enabled at boot)"
ssh "$TARGET" "sed -e 's|@USER@|${PI_USER}|g' -e 's|@DIR@|${PI_DIR}|g' '${PI_DIR}/deploy/rov-pi.service' \
    | sudo tee /etc/systemd/system/rov-pi.service > /dev/null && sudo systemctl daemon-reload"

if [ "$restart" = true ]; then
    echo ">> Restarting rov-pi (this arms the vehicle)"
    ssh "$TARGET" "sudo systemctl restart rov-pi && systemctl --no-pager status rov-pi | head -n 5"
fi

echo ">> Done"
