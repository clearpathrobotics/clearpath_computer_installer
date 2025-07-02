#!/bin/bash

# Check if we're running as root; we should not be
current_user="$(whoami)"
if [ "${current_user}" == "root" ];
then
    echo "This script must not be run as root"
    exit 1
else
    echo "Running as ${current_user}"
fi

# Check OS version and architecture
# Should be 22.04 and aarch64
ubuntu_release="$(lsb_release -r -s | tail -1)"
if ! [ "${ubuntu_release}" == "22.04" ];
then
    echo "This script must be run on Ubuntu 22.04. Detected ${ubuntu_release}"
    exit 1
fi
architecture="$(arch)"
if ! [ "${architecture}" == "aarch64" ];
then
    echo "This script must be run on aarch64. Detected ${architecture}"
    exit 1
fi

# Enable ROS sources
# https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html
echo "Enabling ROS software sources..."
sudo apt-get install -y software-properties-common
sudo add-apt-repository universe
sudo apt-get update
sudo apt install -y curl
export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}')
curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo $VERSION_CODENAME)_all.deb" # If using Ubuntu derivates use $UBUNTU_CODENAME
sudo dpkg -i /tmp/ros2-apt-source.deb


# Enable Clearpath sources
# https://packages.clearpathrobotics.com
echo "Enabling Clearpath software sources..."
wget https://packages.clearpathrobotics.com/public.key -O - | sudo apt-key add -
sudo sh -c 'echo "deb https://packages.clearpathrobotics.com/stable/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/clearpath-latest.list'

# Install initial dependencies
echo "Installing core ROS components and build tools..."
sudo apt-get update
sudo apt-get install -y python3-rosdep python3-colcon-common-extensions ros-dev-tools ros-humble-ros-base
if [ -f /opt/ros/humble/setup.bash ];
then
    source /opt/ros/humble/setup.bash
else
    echo "/opt/ros/humble/setup.bash does not exist. Did ros_base install correctly?"
    exit 1
fi

# Rosdep
echo "Setting up rosdep..."
sudo rosdep init
sudo wget https://raw.githubusercontent.com/clearpathrobotics/public-rosdistro/master/rosdep/50-clearpath.list -O /etc/ros/rosdep/sources.list.d/50-clearpath.list
rosdep update

# ROS base
sudo apt-get install ros-humble-ros-base


# Workspace
mkdir -p $HOME/colcon_ws/src
cd $HOME/colcon_ws/src

