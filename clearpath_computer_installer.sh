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


prompt_option() {
  # ask the user to select from a numbered list of options & return their selection
  # $1 is the variable into which the result is returned
  # $2 should be the question to ask the user as a prompt
  # $3+ should be the available options

  local __resultvar=$1
  shift
  local __prompt=$1
  shift
  local __n_options=$#

  echo -e "\e[39m$__prompt\e[0m"
  for (( i=1; $i<=$__n_options; i++ ));
  do
    opt=${!i}
    echo -e "[$i] \e[32m$opt\e[0m"
  done

  read answer
  eval $__resultvar="'$answer'"
}

prompt_YESno() {
  # as the user a Y/n question
  # $1 is the variable into which the answer is saved as either "n" or "y"
  # $2 is the question to ask

  local __resultvar=$1
  local __prompt=$2

  echo -e "\e[39m$__prompt\e[0m"
  echo "Y/n: "

  if [[ $AUTO_YES == 1 ]];
  then
    echo "Automatically answering Yes"
    eval $__resultvar="y"
  else
    read answer
    if [[ $answer =~ ^[n,N].* ]];
    then
      eval $__resultvar="n"
    else
      eval $__resultvar="y"
    fi
  fi
}

prompt_yesNO() {
  # as the user a y/N question
  # $1 is the variable into which the answer is saved as either "n" or "y"
  # $2 is the question to ask

  local __resultvar=$1
  local __prompt=$2

  echo -e "\e[39m$__prompt\e[0m"
  echo "y/N: "

  if [[ $AUTO_YES == 1 ]];
  then
    echo "Automatically answering No"
    eval $__resultvar="n"
  else
    read answer
    if [[ $answer =~ ^[y,Y].* ]];
    then
      eval $__resultvar="y"
    else
      eval $__resultvar="n"
    fi
  fi
}

# available robots; pre-load the user-choice with -1 to indicate undefined
ROBOT_HUSKY_A200=1
ROBOT_JACKAL_J100=2
ROBOT_WARTHOG_W200=3
ROBOT_RIDGEBACK_R100=4
ROBOT_DINGO_DD100=5
ROBOT_DINGO_DD150=6
ROBOT_DINGO_DO100=7
ROBOT_DINGO_DO150=8
ROBOT_CHOICE=-1

# Set front end to non-interactive to avoid prompts while installing packages
export DEBIAN_FRONTEND=noninteractive

echo ""
echo -e "\e[32mStarting Clearpath Computer Installer\e[0m"
echo ""

# Check if the script is run as root
if [ "$EUID" -eq 0 ]; then
    echo "You are the root user, this needs to be ran as a user to be completed."
fi

# Temporarily disable the blocking messages about restarting services in systems with needrestart installed
if [ -d /etc/needrestart/conf.d ]; then
  sudo bash -c "echo '\$nrconf{restart} = '\''a'\'';' > /etc/needrestart/conf.d/10-auto-cp.conf"
fi

# Get the Ubuntu version
UBUNTU_VERSION=$(. /etc/os-release && echo $UBUNTU_CODENAME)

# Determine the ROS 2 version based on the OS
if [[ "$UBUNTU_VERSION" == "jammy" ]]; then
  ROS_VERSION="humble"
elif [[ "$UBUNTU_VERSION" == "noble" ]]; then
  ROS_VERSION="jazzy"
else
  ROS_VERSION="unsupported"
fi

# Exit the script if the ROS version is unsupported
if [[ "$ROS_VERSION" == "unsupported" ]]; then
  echo -e "\e[31mError: Ubuntu version ($UBUNTU_VERSION) does not have a supported version of ROS 2, exiting\e[0m"
  exit 0
fi

echo -e "\e[94mSetup Open Robotics package server to install ROS 2 $ROS_VERSION\e[0m"

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
sudo apt install iw ros-$ROS_VERSION-ros-base python3-argcomplete ros-dev-tools python3-vcstool ros-$ROS_VERSION-clearpath-robot python3-clearpath-computer-setup -y
echo -e "\e[32mDone: Updating packages and installing ROS 2\e[0m"
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

echo -e "\e[32mDone: Configuring rosdep\e[0m"
echo ""

echo -e "\e[94mConfiguring network service, if needed\e[0m"
# Check if the service file exists
if [ -e "/lib/systemd/system/systemd-networkd-wait-online.service" ]; then
    # Check if timeout is present in the service file
    if grep -q "timeout=30" "/lib/systemd/system/systemd-networkd-wait-online.service"; then
        echo "Timeout is already present in /lib/systemd/system/systemd-networkd-wait-online.service"
    else
        # Add --timeout=30 after ExecStart=/lib/systemd/systemd-networkd-wait-online
        sudo sed -i '/^ExecStart/ s/$/ --timeout=30/' "/lib/systemd/system/systemd-networkd-wait-online.service"
        echo "Timeout added to /lib/systemd/system/systemd-networkd-wait-online.service"
    fi
else
    echo "Service file /lib/systemd/system/systemd-networkd-wait-online.service not found."
fi
# Ensure the service is enabled
sudo systemctl enable systemd-networkd-wait-online

echo -e "\e[32mDone: Configuring network service, if needed\e[0m"
echo ""


### USER ONLY SECTION
if [ ! "$EUID" -eq 0 ]; then

  echo -e "\e[94mUpdating rosdep\e[0m"
  rosdep -q update
  echo -e "\e[32mDone: Updating rosdep\e[0m"
  echo ""

  if [[ $ROBOT_CHOICE -eq -1 ]];
  then
    echo ""
    prompt_option ROBOT_CHOICE "Which robot are you installing?" "Clearpath Husky A200" "Clearpath Jackal J100" "Clearpath Warthog W200" "Clearpath Ridgeback R100" "Clearpath Dingo-D DD100" "Clearpath Dingo-D DD150" "Clearpath Dingo-O DO100" "Clearpath Dingo-O DO150"
  fi
  case "$ROBOT_CHOICE" in
    $ROBOT_HUSKY_A200)
      platform="a200"
      ;;
    $ROBOT_JACKAL_J100)
      platform="j100"
      ;;
    $ROBOT_WARTHOG_W200)
      platform="w200"
      ;;
    $ROBOT_RIDGEBACK_R100)
      platform="r100"
      ;;
    $ROBOT_DINGO_DD100)
      platform="dd100"
      ;;
    $ROBOT_DINGO_DD150)
      platform="dd150"
      ;;
    $ROBOT_DINGO_DO100)
      platform="do100"
      ;;
    $ROBOT_DINGO_DO150)
      platform="do150"
      ;;
    * )
      echo -e "\e[31mERROR: Invalid selection"
      exit 1
      ;;
  esac
  echo "Selected ${platform}."
  echo ""

  # Check if Clearpath folder exists
  if [ -d /etc/clearpath/ ]; then
    echo -e "\e[33mWarn: Clearpath folder exist, skipping\e[0m"
  else
    echo -e "\e[94mCreating setup folder\e[0m"
    sudo mkdir -p -m 777 /etc/clearpath/
    # Check if directory was created
    if [ !  -d /etc/clearpath/ ]; then
      echo -e "\e[31mError: Clearpath folder setup, exiting\e[0m"
      exit 0
    fi
  fi

  # Check if Clearpath Config YAML exists
  if [ -e /etc/clearpath/robot.yaml ]; then
    echo -e "\e[33mWarn: Clearpath Robot YAML exists\e[0m"
    prompt_YESno update_config "\e[39mWould you like to change Clearpath Robot YAML?\e[0m"
    if [[ $update_config == "y" ]]; then
      sudo mv /etc/clearpath/robot.yaml /etc/clearpath/robot.yaml.bkup.$(date +"%Y%m%d%H%M%S")
      echo -e "\e[94mCreating default robot YAML for ${platform}\e[0m"
      sudo cp /opt/ros/$ROS_VERSION/share/clearpath_config/sample/${platform}_default.yaml /etc/clearpath/robot.yaml
      # Check if sources were added
      if [ ! -e /etc/clearpath/robot.yaml ]; then
        echo -e "\e[31mError: Clearpath robot YAML, exiting\e[0m"
        exit 0
      fi
    else
      echo "No change to Clearpath Robot YAML"
    fi
  else
    echo -e "\e[94mCreating default robot YAML for ${platform}\e[0m"
    sudo cp /opt/ros/$ROS_VERSION/share/clearpath_config/sample/${platform}_default.yaml /etc/clearpath/robot.yaml
    sudo chown "$(id -u -n):$(id -g -n)" /etc/clearpath/robot.yaml
    # Check if sources were added
    if [ ! -e /etc/clearpath/robot.yaml ]; then
      echo -e "\e[31mError: Clearpath robot YAML, exiting\e[0m"
      exit 0
    fi
  fi

  echo -e "\e[32mDone: Configuring Clearpath Setup\e[0m"
  echo ""

  while true; do
    echo "Please enter the serial number of the robot (Only 4 digits, the platform model will be automatically added):"
    read serial_number
    # Regular expression to match 4 numbers
    pattern='^[0-9]{4}$'

    if [[ $serial_number =~ $pattern ]]; then
        echo "Serial number is in the correct format."
        break
    else
        echo -e "\e[31mError: Serial number is not in the correct format. It should consist of exactly 4 numbers.\e[0m"
    fi
  done

  platform_serial_number="$platform-$serial_number"
  hostname_string="cpr-$platform-$serial_number"
  platform_namespace="$platform"_"$serial_number"

  # Original file name
  file_name="/etc/clearpath/robot.yaml"

  # Check if the file exists
  if [ -f "$file_name" ]; then
      # Read the content of the file
      original_content=$(<"$file_name")

      # Replace everything after the colon with the new serial number
      updated_content=$(echo "$original_content" | sed "s/serial_number: .*/serial_number: $platform_serial_number/")
      updated_content=$(echo "$updated_content" | sed "s/namespace: .*/namespace: $platform_namespace/")
      updated_content=$(echo "$updated_content" | sed "s/hostname: .*/hostname: $hostname_string/")

      # Write the updated content back to the file
      sudo echo "$updated_content" > "$file_name"

      echo "Serial number updated in $file_name."
  else
        echo -e "\e[31mError: File $file_name does not exist.\e[0m"
  fi


  # Check if the hostname is cpr-unassigned
  echo -e "\e[94mChecking hostname\e[0m"
  if [ "$(hostname)" = "clearpath-unassigned" ]; then
    echo "Hostname is currently set to 'clearpath-unassigned'."
    sudo hostnamectl set-hostname "$hostname_string"
    # Display the new hostname
    echo "Hostname changed to '$hostname_string'."
    # Notify the user to restart for changes to take effect
    echo "Please restart your system for the changes to take effect."
  else
      echo "Hostname is already set to '$(hostname)'. No changes needed."
  fi
  echo -e "\e[32mDone: Checking hostname\e[0m"
  echo ""

  source /opt/ros/$ROS_VERSION/setup.bash

  prompt_YESno install_service "\e[39mWould you like to install Clearpath services?\e[0m"
  if [[ $install_service == "y" ]]; then
    echo -e "\e[94mInstalling clearpath robot service\e[0m"
    ros2 run clearpath_robot install

    if [ $? -eq 0 ]; then
      echo -e "\e[32mDone: Installing clearpath robot service\e[0m"
      echo ""
    else
      echo -e "\e[31mError: Failed to install clearpath robot service\e[0m"
      exit 0
    fi
  else
    echo "Skipping installing Clearpath services"
  fi

  sudo systemctl enable clearpath-robot

  echo -e "\e[94mSetting up clearpath enviroment\e[0m"
  grep -qxF "source /etc/clearpath/setup.bash" ~/.bashrc || echo "source /etc/clearpath/setup.bash" >> ~/.bashrc
  echo -e "\e[32mDone: Setting up clearpath enviroment\e[0m"
  echo ""

  echo -e "\e[94mSetting up groups\e[0m"

  if [ $(getent group video) ];
  then
    echo "video group already exists";
  else
    echo "Adding video group";
    sudo addgroup video;
  fi
  if id -nGz "$(whoami)" | grep -qzxF "video";
  then
    echo "User:$(whoami) is already in video group";
  else
    echo "Adding user:$(whoami) to video group";
    sudo usermod -a -G video $(whoami);
  fi

  if [ $(getent group flirimaging) ];
  then
    echo "flirimaging group already exists";
  else
    echo "Adding flirimaging group";
    sudo addgroup flirimaging;
  fi
  if id -nGz "$(whoami)" | grep -qzxF "flirimaging";
  then
    echo "User:$(whoami) is already in flirimaging group";
  else
    echo "Adding user:$(whoami) to flirimaging group";
    sudo usermod -a -G flirimaging $(whoami);
  fi

  val=$(< /sys/module/usbcore/parameters/usbfs_memory_mb)
  if [ "$val" -lt "2048" ]; then
    if [ -e /etc/default/grub ]; then
      if [ $(grep -c "usbcore.usbfs_memory_mb=" /etc/default/grub) -eq 0 ]; then # Memory Limit has not already been set
        sudo sed -i 's/GRUB_CMDLINE_LINUX_DEFAULT="[^"]*/& usbcore.usbfs_memory_mb=2048/' /etc/default/grub
        echo "Increased the usbfs memory limits in the default grub configuration. Updating grub"
        sudo update-grub
      else
        echo -e "\e[33mWarn: usbfs memory limit is already set in /etc/default/grub in the following line:\e[0m"
        echo "$(grep "usbcore.usbfs_memory_mb" /etc/default/grub)"
        echo -e "\e[33mNo changes made, verify that usbfs_memory_mb is set to a minimum of 2048 and then try rebooting the computer\e[0m"
      fi

    else
      echo -e "\e[33mWarn: /etc/default/grub configuration file not found, no changes made. usbfs_memory_mb must be set manually.\e[0m"
      echo -e "\e[33mSee https://github.com/ros-drivers/flir_camera_driver/tree/humble-release/spinnaker_camera_driver#setting-up-linux-without-spinnaker-sdk for instructions\e[0m"
      exit 0
    fi
  else
    echo "usbfs_memory_mb is already set to $val, no changes necessary."
  fi

  echo -e "\e[32mDone: Setting up groups\e[0m"
  echo ""
  echo -e "\e[32mClearpath Computer Installer Complete\e[0m"
  echo -e "\e[94mTo continue installation visit: https://docs.clearpathrobotics.com/docs/ros/networking/computer_setup \e[0m"
  echo ""
else
  echo -e "\e[32mClearpath Computer Installer needs to be ran as a user, please re-run.\e[0m"
  echo ""
fi



# Reenable messages about restarting services in systems with needrestart installed
if [ -e /etc/needrestart/conf.d/10-auto-cp.conf ]; then
  sudo rm /etc/needrestart/conf.d/10-auto-cp.conf
fi


if ping -c1 gitlab.clearpathrobotics.com;
then
  echo -e "\e[94mDownloading wireless configuration script for use later\e[0m"
  wget https://gitlab.clearpathrobotics.com/research/lv426-netplan/-/raw/main/configure-lv426.sh -O /home/$USER/setup-lv426.sh
  chmod +x /home/$USER/setup-lv426.sh
fi
