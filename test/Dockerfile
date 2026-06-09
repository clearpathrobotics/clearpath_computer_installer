# Systemd-enabled Ubuntu image for testing clearpath_computer_installer.sh.
#
# The installer relies on systemd (systemctl enable / daemon-reload / hostnamectl)
# and must run as a non-root user with passwordless sudo. This image therefore
# boots real systemd as PID 1 and provides such a user.
#
# Build from the repository ROOT (so the installer scripts are in the context):
#   docker build --build-arg UBUNTU_CODENAME=noble -t cpr-installer-test:noble -f test/Dockerfile .
#
# Supported codenames: jammy (ROS 2 Humble), noble (ROS 2 Jazzy).
ARG UBUNTU_CODENAME=noble
FROM ubuntu:${UBUNTU_CODENAME}

ENV DEBIAN_FRONTEND=noninteractive
ENV container=docker

# Pre-configure tzdata so it never prompts (systemd boots a fresh env that
# does not inherit the DEBIAN_FRONTEND=noninteractive set above).
RUN ln -fs /usr/share/zoneinfo/Etc/UTC /etc/localtime

# Minimal tooling the installer expects to exist early, plus systemd + sudo.
# NOTE: apt lists are intentionally NOT purged. The installer installs
# software-properties-common before its first `apt update`, which mirrors a
# fresh Ubuntu machine where the package lists already exist.
RUN apt-get update && apt-get install -y --no-install-recommends \
        systemd systemd-sysv sudo ca-certificates locales lsb-release \
        git curl wget gnupg pciutils iputils-ping

# Locale (ROS tooling expects a UTF-8 locale).
RUN locale-gen en_US en_US.UTF-8 && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
ENV LANG=en_US.UTF-8
ENV LC_ALL=en_US.UTF-8

# Non-root user with passwordless sudo (installer must NOT run as root).
# Defaults to 'robot' to match system.username in robot.yaml and the
# User=robot directive in the installed clearpath-*.service units.
ARG USERNAME=robot
RUN useradd --create-home --shell /bin/bash "${USERNAME}" \
    && echo "${USERNAME} ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/${USERNAME} \
    && chmod 0440 /etc/sudoers.d/${USERNAME}

# Mask systemd units that are pointless or fail inside a container so the
# system reaches a "running"/"degraded" state quickly.
RUN systemctl mask \
        systemd-udevd.service \
        systemd-udevd-kernel.socket \
        systemd-udevd-control.socket \
        systemd-modules-load.service \
        sys-kernel-config.mount \
        sys-kernel-debug.mount \
        getty.target \
        console-getty.service

# Bake the installer scripts into the image (you can also bind-mount instead).
COPY clearpath_computer_installer.sh cockpit_installer.sh /home/${USERNAME}/installer/
RUN chown -R ${USERNAME}:${USERNAME} /home/${USERNAME}/installer

STOPSIGNAL SIGRTMIN+3
CMD ["/lib/systemd/systemd"]
