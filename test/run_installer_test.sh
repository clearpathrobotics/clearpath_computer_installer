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
#   ROBOT_CHOICE   Numeric platform selection fed to the installer's first
#                  prompt (the installer hardcodes its internal default, so this
#                  is supplied via stdin, not the environment).
#                  1=A300 2=A200 3=J100(default) 4=W200 5=R100
#                  6=DD100 7=DD150 8=DO100 9=DO150
#   SERIAL         Robot serial digits fed to the installer prompt.
#                  4 digits for most platforms, 5 digits for A300. Default 0001.
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
ROBOT_CHOICE="${ROBOT_CHOICE:-3}"
SERIAL="${SERIAL:-0001}"
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
# The installer reads the platform selection and the serial number from stdin
# (the platform choice cannot be set via the environment), so feed both lines.
set +e
docker exec -i -u "${USERNAME}" \
    -e AUTO_YES=1 \
    "${CONTAINER}" \
    bash -lc 'cd ~/installer && printf "%s\n%s\n" "'"${ROBOT_CHOICE}"'" "'"${SERIAL}"'" | bash -e clearpath_computer_installer.sh'
rc=$?
set -e

echo "==> Installer exited with code ${rc}"
echo "==> Verifying the network-timeout drop-in was created"
docker exec "${CONTAINER}" bash -lc \
    'cat /etc/systemd/system/systemd-networkd-wait-online.service.d/override.conf 2>/dev/null \
        && echo "drop-in OK" || echo "drop-in MISSING"'

if [[ "${KEEP:-0}" == "1" ]]; then
    echo "==> KEEP=1 set; container left running. Inspect with:"
    echo "    docker exec -it ${CONTAINER} bash"
else
    echo "==> Cleaning up container ${CONTAINER}"
    docker rm -f "${CONTAINER}" >/dev/null
fi

exit "${rc}"
