#!/bin/bash -e
# Software License Agreement (BSD)
#
# Author    Tony Baltovski <tbaltovski@clearpathrobotics.com>
# Copyright (c) 2022, Clearpath Robotics, Inc., All rights reserved.
#
# Redistribution and use in source and binary forms, with or without modification, are permitted provided that
# the following conditions are met:
# * Redistributions of source code must retain the above copyright notice, this list of conditions and the
#   following disclaimer.
# * Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the
#   following disclaimer in the documentation and/or other materials provided with the distribution.
# * Neither the name of Clearpath Robotics nor the names of its contributors may be used to endorse or
#   promote products derived from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED
# WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
# PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
# ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
# TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
# HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
# NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#

echo ""
echo -e "\e[32mStarting Clearpath Computer Installer\e[0m"
echo ""

# Temporarily disable the blocking messages about restarting services in systems with needrestart installed
if [ -d /etc/needrestart/conf.d ]; then
  sudo bash -c "echo '\$nrconf{restart} = '\''a'\'';' > /etc/needrestart/conf.d/10-auto-cp.conf"
fi

echo -e "\e[94mSetup Open Robotics package server to install ROS 2 Humble\e[0m"

# Check if ROS 2 sources are already installed
if [ -e /etc/apt/sources.list.d/ros2.list ]; then
  echo -e "\e[33mWarn: ROS 2 sources exist, skipping\e[0m"
else
  sudo apt -y -qq install software-properties-common
  sudo add-apt-repository universe -y
  sudo apt -y -qq update && sudo apt -y -qq upgrade && sudo apt -y -qq install curl -y
  sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
  echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
  # Check if sources were added
  if [ ! -e /etc/apt/sources.list.d/ros2.list ]; then
    echo -e "\e[31mError: Unable to add ROS 2 package server, exiting\e[0m"
    exit 0
  fi
fi

echo -e "\e[32mDone: Setup ROS 2 package server\e[0m"
echo ""

echo -e "\e[94mSetup Clearpath Robotics package server\e[0m"

# Check if Clearpath sources are already installed
if [ -e /etc/apt/sources.list.d/clearpath-latest.list ]; then
  echo -e "\e[33mWarn: Clearpath Robotics sources exist, skipping\e[0m"
else
  wget https://packages.clearpathrobotics.com/public.key -O - | sudo apt-key add -
  sudo bash -c 'echo "deb https://packages.clearpathrobotics.com/stable/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/clearpath-latest.list'
  # Check if sources were added
  if [ ! -e /etc/apt/sources.list.d/clearpath-latest.list ]; then
    echo -e "\e[31mError: Unable to add Clearpath Robotics package server, exiting\e[0m"
    exit 0
  fi
fi

echo -e "\e[32mDone: Setup Clearpath Robotics package server\e[0m"
echo ""


echo -e "\e[94mUpdating packages and installing ROS 2\e[0m"
sudo apt -y -qq update
sudo apt install ros-humble-ros-base python3-argcomplete ros-dev-tools python3-vcstool ros-humble-clearpath-robot python3-clearpath-computer-setup -y
echo -e "\e[32mDone: Updating packages and installing ROS 2\e[0m"
echo ""

echo -e "\e[94mSetting up enviroment\e[0m"
grep -qxF 'source /opt/ros/humble/setup.bash' ~/.bashrc || echo 'source /opt/ros/humble/setup.bash' >> ~/.bashrc
source /opt/ros/humble/setup.bash
echo -e "\e[32mDone: Setting up enviroment\e[0m"
echo ""

echo -e "\e[94mConfiguring rosdep\e[0m"

# Check if rosdep sources are already installed
if [ -e /etc/ros/rosdep/sources.list.d/20-default.list ]; then
  echo -e "\e[33mWarn: rosdep was initalized, skipping\e[0m"
else
  sudo rosdep -q init
  # Check if sources were added
  if [ ! -e /etc/ros/rosdep/sources.list.d/20-default.list ]; then
    echo -e "\e[31mError: rosdep failed to initalize, exiting\e[0m"
    exit 0
  fi
fi

# Check if Clearpath rosdep sources are already installed
if [ -e /etc/ros/rosdep/sources.list.d/50-clearpath.list ]; then
  echo -e "\e[33mWarn: CPR rosdeps exist, skipping\e[0m"
else
  sudo wget -q https://raw.githubusercontent.com/clearpathrobotics/public-rosdistro/master/rosdep/50-clearpath.list -O \
    /etc/ros/rosdep/sources.list.d/50-clearpath.list
  # Check if sources were added
  if [ ! -e /etc/ros/rosdep/sources.list.d/50-clearpath.list ]; then
    echo -e "\e[31mError: CPR rosdeps, exiting\e[0m"
    exit 0
  fi
fi

rosdep -q update
echo -e "\e[32mDone: Configuring rosdep\e[0m"
echo ""


echo -e "\e[94mSetting up groups\e[0m"
sudo addgroup flirimaging
sudo usermod -a -G flirimaging administrator
sudo sed -i 's/GRUB_CMDLINE_LINUX_DEFAULT="[^"]*/& usbcore.usbfs_memory_mb=1000/' /etc/default/grub
sudo update-grub
echo -e "\e[32mDone: Setting up groups\e[0m"
echo ""


echo -e "\e[94mCreating setup folder\e[0m"
sudo mkdir -p -m 777 /etc/clearpath/
sudo wget -q https://raw.githubusercontent.com/clearpathrobotics/clearpath_config/main/clearpath_config/sample/a200/a200_default.yaml -O /etc/clearpath/robot.yaml
echo -e "\e[32mDone: Creating setup folder\e[0m"
echo ""

echo -e "\e[94mInstalling clearpath robot service\e[0m"
source /opt/ros/humble/setup.bash
ros2 run clearpath_robot install
if [ $? -eq 0 ]; then
  echo -e "\e[32mDone: Installing clearpath robot service\e[0m"
  echo ""
else
    echo -e "\e[31mError: Failed to install clearpath robot service\e[0m"
    exit 0
fi

echo -e "\e[94mSetting up clearpath enviroment\e[0m"
grep -qxF 'source /etc/clearpath/setup.bash' ~/.bashrc || echo 'source /etc/clearpath/setup.bash' >> ~/.bashrc
echo -e "\e[32mDone: Setting up clearpath enviroment\e[0m"
echo ""

# Reenable messages about restarting services in systems with needrestart installed
if [ -d /etc/needrestart/conf.d ]; then
  sudo rm /etc/needrestart/conf.d/10-auto-cp.conf
fi

echo -e "\e[32mClearpath Computer Installer Complete\e[0m"
echo -e "\e[94mTo continue installation visit: https://docs.clearpathrobotics.com/docs/ros/networking/computer_setup \e[0m"
echo -e "\e[94mTo start the robot service run:\e[0m sudo systemctl daemon-reload && sudo systemctl start clearpath-robot.service"
