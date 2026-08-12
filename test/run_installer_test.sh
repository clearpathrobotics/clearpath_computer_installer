#!/usr/bin/env bash
#
# Build a systemd-enabled container and run clearpath_computer_installer.sh
# inside it to exercise the install process end-to-end.
#
# Usage:
#   test/run_installer_test.sh [UBUNTU_CODENAME]
#
#   UBUNTU_CODENAME   noble (Jazzy, default) or jammy (Humble)
#
# Environment overrides:
#   ROBOT_CHOICE   Numeric platform selection passed to the installer via the
#                  ROBOT_CHOICE environment variable.
#                  1=A300 2=A200 3=J100 4=W200 5=R100
#                  6=DD100 7=DD150 8=DO100 9=DO150
#                  Default: 1 (A300) on noble, 3 (J100) on jammy.
#   SERIAL         Robot serial digits passed via the SERIAL_NUMBER env var.
#                  4 digits for most platforms, 5 digits for A300.
#                  Default: 00001 on noble, 0001 on jammy.
#   KEEP           If set to 1, leave the container running for inspection.
#
# Notes:
#   * Requires Docker with cgroup v2 (modern Docker on Linux).
#   * The container runs --privileged because systemd needs it.
#   * Hardware-specific tail steps (GRUB usbfs tuning, NVIDIA/CUDA detection)
#     are expected to be skipped or to warn inside a container; the meaningful
#     package install, systemd network-timeout drop-in, and clearpath-robot
#     service install all run before that point.
set -euo pipefail

CODENAME="${1:-noble}"

# A300 is only supported on Jazzy (noble); fall back to J100 on Humble (jammy).
if [[ "${CODENAME}" == "noble" ]]; then
  ROBOT_CHOICE="${ROBOT_CHOICE:-1}"   # A300
  SERIAL="${SERIAL:-00001}"            # 5-digit serial
else
  ROBOT_CHOICE="${ROBOT_CHOICE:-3}"   # J100
  SERIAL="${SERIAL:-0001}"             # 4-digit serial
fi

# Must match system.username in robot.yaml (default 'robot'), which the
# clearpath-*.service units run as (User=robot).
USERNAME="robot"
IMAGE="cpr-installer-test:${CODENAME}"
CONTAINER="cpr-installer-test-${CODENAME}"

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

echo "==> Building image ${IMAGE} (Ubuntu ${CODENAME})"
docker build \
    --build-arg "UBUNTU_CODENAME=${CODENAME}" \
    -t "${IMAGE}" \
    -f "${REPO_ROOT}/test/Dockerfile" \
    "${REPO_ROOT}"

echo "==> Removing any previous container named ${CONTAINER}"
docker rm -f "${CONTAINER}" >/dev/null 2>&1 || true

echo "==> Starting systemd container ${CONTAINER}"
docker run -d --name "${CONTAINER}" \
    --privileged \
    --cgroupns=host \
    --tmpfs /run \
    --tmpfs /run/lock \
    -v /sys/fs/cgroup:/sys/fs/cgroup:rw \
    "${IMAGE}" >/dev/null

echo "==> Waiting for systemd to finish booting"
for _ in $(seq 1 60); do
    state="$(docker exec "${CONTAINER}" systemctl is-system-running 2>/dev/null || true)"
    case "${state}" in
        running|degraded) echo "    systemd state: ${state}"; break ;;
        *) sleep 1 ;;
    esac
done

echo "==> Running installer (ROBOT_CHOICE=${ROBOT_CHOICE}, SERIAL=${SERIAL})"
# Inputs are passed via environment variables (ROBOT_CHOICE / SERIAL_NUMBER /
# AUTO_YES) so the run is fully non-interactive. stdin is redirected from
# /dev/null so any stray read returns immediately instead of hanging.
set +e
docker exec -u "${USERNAME}" \
    -e DEBIAN_FRONTEND=noninteractive \
    -e AUTO_YES=1 \
    -e ROBOT_CHOICE="${ROBOT_CHOICE}" \
    -e SERIAL_NUMBER="${SERIAL}" \
    "${CONTAINER}" \
    bash -lc 'cd ~/installer && bash -e clearpath_computer_installer.sh < /dev/null'
rc=$?
set -e

echo "==> Installer exited with code ${rc}"
echo "==> Verifying the network-timeout drop-in was created"
docker exec "${CONTAINER}" bash -lc \
    'cat /etc/systemd/system/systemd-networkd-wait-online.service.d/99-wait-any.conf 2>/dev/null \
        && echo "drop-in OK" || echo "drop-in MISSING"'

if [[ "${KEEP:-0}" == "1" ]]; then
    echo "==> KEEP=1 set; container left running. Inspect with:"
    echo "    docker exec -it ${CONTAINER} bash"
else
    echo "==> Cleaning up container ${CONTAINER}"
    docker rm -f "${CONTAINER}" >/dev/null
fi

exit "${rc}"
