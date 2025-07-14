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


# Text formatting variables
RESET_TEXT='\e[0m'
INFO_BLUE='\e[94mINFO: '
DONE_GREEN='\e[32mDONE: '
ERROR_RED='\e[31mERROR: '
WARN_YELLOW='\e[33mWARN: '

#Log functions
log_space() {
  # Print a blank line
  echo ""
}

log_info() {
  # Print the information message
  echo -e "${INFO_BLUE}$1${RESET_TEXT}"
}

log_warn() {
  # Print the warning message
  echo -e "${WARN_YELLOW}$1${RESET_TEXT}"
}

log_error() {
  # Print the error message
  echo -e "${ERROR_RED}$1${RESET_TEXT}"
}

log_done() {
  # Print the error message
  echo -e "${DONE_GREEN}$1${RESET_TEXT}"
  log_space
}


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
ROBOT_HUSKY_A300=1
ROBOT_HUSKY_A200=2
ROBOT_JACKAL_J100=3
ROBOT_WARTHOG_W200=4
ROBOT_RIDGEBACK_R100=5
ROBOT_DINGO_DD100=6
ROBOT_DINGO_DD150=7
ROBOT_DINGO_DO100=8
ROBOT_DINGO_DO150=9
ROBOT_CHOICE=-1

# Get the platform model from the user
step_get_platform_model() {
  if [[ $ROBOT_CHOICE -eq -1 ]];
  then
    echo ""
    prompt_option ROBOT_CHOICE "Which robot platform are you installing?" "Clearpath Husky A300" "Clearpath Husky A200" "Clearpath Jackal J100" "Clearpath Warthog W200" "Clearpath Ridgeback R100" "Clearpath Dingo-D DD100" "Clearpath Dingo-D DD150" "Clearpath Dingo-O DO100" "Clearpath Dingo-O DO150"
  fi
  case "$ROBOT_CHOICE" in
    $ROBOT_HUSKY_A300)
      platform="a300"
      ;;
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
      log_error "Invalid selection"
      exit 1
      ;;
  esac
  echo "Selected platform: ${platform}."
  log_space
}


# Determine the OS and ROS version
step_get_os_and_ros_version() {
  # Get the Ubuntu version
  UBUNTU_VERSION=$(. /etc/os-release && echo $UBUNTU_CODENAME)

  # Determine the ROS 2 version based on the OS
  if [[ "$UBUNTU_VERSION" == "jammy" ]]; then
    ROS_DISTRO_MANUAL="humble"
  elif [[ "$UBUNTU_VERSION" == "noble" ]]; then
    ROS_DISTRO_MANUAL="jazzy"
  else
    ROS_DISTRO_MANUAL="unsupported"
  fi

  # Exit the script if the ROS version is unsupported
  if [[ "$ROS_DISTRO_MANUAL" == "unsupported" ]]; then
    log_error "Ubuntu version ($UBUNTU_VERSION) does not have a supported version of ROS 2, exiting"
    exit 0
  fi
}

# Setup Open Robotics package server to install ROS 2
step_setup_osrf_packge_server() {
  log_info "Setup Open Robotics package server to install ROS 2 $ROS_DISTRO_MANUAL"

  if [ -e /etc/apt/sources.list.d/ros2.list ]; then
    # Remove old ROS 2 installation if present
    # See https://discourse.ros.org/t/ros-signing-key-migration-guide/43937
    log_info "Detected old ROS 2 installation"
    sudo rm /etc/apt/sources.list.d/ros2.list
    sudo rm /usr/share/keyrings/ros-archive-keyring.gpg
  fi

  # Check if ROS 2 sources are already installed
  if dpkg -s ros2-apt-source &> /dev/null; then
    log_warn "ROS 2 sources exist, skipping"
  else
    sudo apt -y -qq install software-properties-common
    sudo add-apt-repository universe -y
    sudo apt -y -qq update && sudo apt -y -qq upgrade && sudo apt -y -qq install curl -y

    # See https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html
    export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}')
    curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo $VERSION_CODENAME)_all.deb"
    sudo apt install /tmp/ros2-apt-source.deb
    sudo rm /tmp/ros2-apt-source.deb

    # Check if sources were added
    if ! dpkg -s ros2-apt-source &> /dev/null; then
      log_error "Unable to add ROS 2 package server, exiting"
      exit 0
    fi
  fi

  log_done "Setup ROS 2 package server"
}

# Setup Clearpath Robotics package server
step_setup_cpr_packge_server() {
  log_info "Setup Clearpath Robotics package server"

  # Check if Clearpath sources are already installed
  if [ -e /etc/apt/sources.list.d/clearpath-latest.list ]; then
    log_warn "Clearpath Robotics sources exist, skipping"
  else
    wget https://packages.clearpathrobotics.com/public.key -O - | sudo apt-key add -
    sudo bash -c 'echo "deb https://packages.clearpathrobotics.com/stable/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/clearpath-latest.list'
    # Check if sources were added
    if [ ! -e /etc/apt/sources.list.d/clearpath-latest.list ]; then
      log_error "Unable to add Clearpath Robotics package server, exiting"
      exit 0
    fi
  fi

  log_done "Setup Clearpath Robotics package server"
}

# Install the ROS 2 packages needed for the given ROS 2 distro
step_install_ros_packages() {
  log_info "Updating packages and installing ROS 2"
  sudo apt -y -qq update
  # All ROS distros
  sudo apt install -y -qq  iw ros-$ROS_DISTRO_MANUAL-ros-base ros-$ROS_DISTRO_MANUAL-clearpath-robot python3-argcomplete ros-dev-tools python3-vcstool python3-ds4drv
  if [[ "$ROS_DISTRO_MANUAL" == "humble" ]]; then
    sudo apt -y -qq  install python3-clearpath-computer-setup
  elif [[ "$ROS_DISTRO_MANUAL" == "jazzy" ]]; then
    sudo apt -y -qq  install ros-jazzy-foxglove-bridge python3-pip openssh-server
    sudo pip install --break-system-packages canopen
  fi
  log_done "Updating packages and installing ROS 2"
}

# Setup rosdep for OSRF and Clearpath packages
step_setup_rosdep() {
  log_info "Configuring rosdep"

  # Check if rosdep sources are already installed
  if [ -e /etc/ros/rosdep/sources.list.d/20-default.list ]; then
    log_warn "rosdep was already initialized, skipping"
  else
    sudo rosdep -q init
    # Check if sources were added
    if [ ! -e /etc/ros/rosdep/sources.list.d/20-default.list ]; then
      log_error "rosdep failed to initialize, exiting"
      exit 0
    fi
  fi

  # Check if Clearpath rosdep sources are already installed
  if [ -e /etc/ros/rosdep/sources.list.d/50-clearpath.list ]; then
    log_warn "Clearpath Robotics rosdeps exist, skipping"
  else
    sudo wget -q https://raw.githubusercontent.com/clearpathrobotics/public-rosdistro/master/rosdep/50-clearpath.list -O \
      /etc/ros/rosdep/sources.list.d/50-clearpath.list
    # Check if sources were added
    if [ ! -e /etc/ros/rosdep/sources.list.d/50-clearpath.list ]; then
      log_error "Clearpath Robotics rosdeps, exiting"
      exit 0
    fi
  fi

  log_done "Configuring rosdep"
}

setup_change_net_timeout() {
  log_info "Configuring network service, if needed"
  # Check if the service file exists
  if [ -e "/lib/systemd/system/systemd-networkd-wait-online.service" ]; then
    # Check if timeout is present in the service file
    if grep -q "timeout=30" "/lib/systemd/system/systemd-networkd-wait-online.service"; then
      log_info "Timeout is already present in /lib/systemd/system/systemd-networkd-wait-online.service"
    else
      # Add --timeout=30 after ExecStart=/lib/systemd/systemd-networkd-wait-online
      sudo sed -i '/^ExecStart/ s/$/ --timeout=30/' "/lib/systemd/system/systemd-networkd-wait-online.service"
      log_info "Timeout added to /lib/systemd/system/systemd-networkd-wait-online.service"
    fi
  else
    log_warn "Service file /lib/systemd/system/systemd-networkd-wait-online.service not found."
  fi
  # Ensure the service is enabled
  sudo systemctl enable systemd-networkd-wait-online

  log_done "Configuring network service, if needed"
}

log_space
log_info "Starting Clearpath Robotics Computer Installer"
log_space

# Get the platform model from the user
step_get_platform_model

# Determine the OS and ROS version
step_get_os_and_ros_version

# Check if A300 since it is only supported on Jazzy
if [[ $platform == "a300" && $ROS_DISTRO_MANUAL == "humble" ]]; then
  log_error "Husky A300 is only supported on ROS 2 Jazzy, exiting"
  exit 1
fi

# Set front end to non-interactive to avoid prompts while installing packages
export DEBIAN_FRONTEND=noninteractive

# Check if the script is run as root
if [ "$EUID" -eq 0 ]; then
 log_warn "You are the root user, this needs to be run as a user to be completed."
fi

# Temporarily disable the blocking messages about restarting services in systems with needrestart installed
if [ -d /etc/needrestart/conf.d ]; then
  sudo bash -c "echo '\$nrconf{restart} = '\''a'\'';' > /etc/needrestart/conf.d/10-auto-cp.conf"
fi

step_setup_osrf_packge_server

step_setup_cpr_packge_server

step_install_ros_packages

step_setup_rosdep

setup_change_net_timeout

### USER ONLY SECTION
if [ ! "$EUID" -eq 0 ]; then

  log_info "Updating rosdep"
  rosdep -q update
  log_done "Updating rosdep"

  # Check if Clearpath folder exists
  if [ -d /etc/clearpath/ ]; then
    log_warn "Clearpath folder exist, skipping"
  else
    log_info "Creating setup folder"
    sudo mkdir -p -m 777 /etc/clearpath/
    # Check if directory was created
    if [ !  -d /etc/clearpath/ ]; then
      log_error "Clearpath folder setup, exiting"
      exit 0
    fi
  fi

  # Check if Clearpath Config YAML exists
  if [ -e /etc/clearpath/robot.yaml ]; then
    log_warn "Clearpath Robot YAML exists"
    prompt_YESno update_config "Would you like to change Clearpath Robot YAML?"
    if [[ $update_config == "y" ]]; then
      sudo mv /etc/clearpath/robot.yaml /etc/clearpath/robot.yaml.backup.$(date +"%Y%m%d%H%M%S")
      log_info "Creating default robot YAML for ${platform}"
      sudo cp /opt/ros/$ROS_DISTRO_MANUAL/share/clearpath_config/sample/${platform}_default.yaml /etc/clearpath/robot.yaml
      # Check if sources were added
      if [ ! -e /etc/clearpath/robot.yaml ]; then
        log_error "Clearpath robot YAML, exiting"
        exit 0
      fi
    else
      echo "No change to Clearpath Robot YAML"
    fi
  else
    log_info "Creating default robot YAML for ${platform}"
    sudo cp /opt/ros/$ROS_DISTRO_MANUAL/share/clearpath_config/sample/${platform}_default.yaml /etc/clearpath/robot.yaml
    sudo chown "$(id -u -n):$(id -g -n)" /etc/clearpath/robot.yaml
    # Check if sources were added
    if [ ! -e /etc/clearpath/robot.yaml ]; then
      log_error "Clearpath robot YAML, exiting"
      exit 0
    fi
  fi

  log_info "Configuring Clearpath Setup"

  while true; do
    # A300 uses 5 digit serial numbers, others uses 4 digit serial numbers
    if [[ $platform == "a300" ]]; then
      log_info "Please enter the serial number of the robot (Only 5 digits, the platform model will be automatically added):"
      read serial_number
      # Regular expression to match 5 numbers
      pattern='^[0-9]{5}$'

      if [[ $serial_number =~ $pattern ]]; then
        log_info "Serial number is in the correct format."
        break
      else
        log_error "Serial number is not in the correct format. It should consist of exactly 5 numbers."
      fi
    else
      log_info "Please enter the serial number of the robot (Only last 4 digits, the platform model will be automatically added):"
      read serial_number
      # Regular expression to match 4 numbers
      pattern='^[0-9]{4}$'

      if [[ $serial_number =~ $pattern ]]; then
        log_info "Serial number is in the correct format."
        break
      else
        log_error "Serial number is not in the correct format. It should consist of exactly 4 numbers."
      fi
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

    log_info "Serial number updated in $file_name."
  else
    log_error "File $file_name does not exist"
  fi


  # Check if the hostname is cpr-unassigned
  log_info "Checking hostname\e[0m"
  if [ "$(hostname)" = "clearpath-unassigned" ]; then
    log_info "Hostname is currently set to 'clearpath-unassigned'."
    sudo hostnamectl set-hostname "$hostname_string"
    # Display the new hostname
    log_info "Hostname changed to '$hostname_string'."
    sudo sed -i "s/clearpath-unassigned/$hostname_string/g" /etc/hosts
    # Notify the user to restart for changes to take effect
    log_info "Please restart your system for the changes to take effect."
  else
      log_info "Hostname is already set to '$(hostname)'. No changes needed."
  fi
  log_done "Checking hostname"

  source /opt/ros/$ROS_DISTRO_MANUAL/setup.bash

  if [[ "$ROS_DISTRO_MANUAL" != "humble" ]]; then
    prompt_YESno install_cockpit "Would you like to install Cockpit webserver for management and diagnostics?"
    if [[ $install_cockpit == "y" ]]; then
      wget -c https://raw.githubusercontent.com/clearpathrobotics/clearpath_computer_installer/main/cockpit_installer.sh && bash -e cockpit_installer.sh
    fi
  fi

  prompt_YESno install_service "Would you like to install Clearpath services?"
  if [[ $install_service == "y" ]]; then
    log_info "Installing clearpath robot service"
    ros2 run clearpath_robot install

    if [ $? -eq 0 ]; then
      log_done "Installing Clearpath robot service"
    else
      log_error "Failed to install Clearpath robot service"
      exit 0
    fi
  else
    log_warn "Skipping installing Clearpath services"
  fi

  sudo systemctl enable clearpath-robot

  log_info "Setting up Clearpath environment"
  grep -qxF "source /etc/clearpath/setup.bash" ~/.bashrc || echo "source /etc/clearpath/setup.bash" >> ~/.bashrc
  log_done "Setting up Clearpath environment"


  log_info "Setting up groups\e[0m"

  if [ $(getent group video) ]; then
    log_info "video group already exists";
  else
    log_info "Adding video group";
    sudo addgroup video;
  fi
  if id -nGz "$(whoami)" | grep -qzxF "video"; then
    log_info "User:$(whoami) is already in video group";
  else
    log_info "Adding user:$(whoami) to video group";
    sudo usermod -a -G video $(whoami);
  fi

  if [ $(getent group flirimaging) ]; then
    log_info "flirimaging group already exists";
  else
    log_info "Adding flirimaging group";
    sudo addgroup flirimaging;
  fi
  if id -nGz "$(whoami)" | grep -qzxF "flirimaging"; then
    log_info "User:$(whoami) is already in flirimaging group";
  else
    log_info "Adding user:$(whoami) to flirimaging group";
    sudo usermod -a -G flirimaging $(whoami);
  fi

  val=$(< /sys/module/usbcore/parameters/usbfs_memory_mb)
  if [ "$val" -lt "2048" ]; then
    if [ -e /etc/default/grub ]; then
      if [ $(grep -c "usbcore.usbfs_memory_mb=" /etc/default/grub) -eq 0 ]; then # Memory Limit has not already been set
        sudo sed -i 's/GRUB_CMDLINE_LINUX_DEFAULT="[^"]*/& usbcore.usbfs_memory_mb=2048/' /etc/default/grub
        log_info "Increased the usbfs memory limits in the default grub configuration. Updating grub"
        sudo update-grub
      else
        log_warn "usbfs memory limit is already set in /etc/default/grub in the following line:\e[0m"
        echo "$(grep "usbcore.usbfs_memory_mb" /etc/default/grub)"
        log_warn "No changes made, verify that usbfs_memory_mb is set to a minimum of 2048 and then try rebooting the computer"
      fi

    else
      log_warn "/etc/default/grub configuration file not found, no changes made. usbfs_memory_mb must be set manually."
      log_warn "See https://github.com/ros-drivers/flir_camera_driver/tree/humble-release/spinnaker_camera_driver#setting-up-linux-without-spinnaker-sdk for instructions"
      exit 0
    fi
  else
    log_info "usbfs_memory_mb is already set to $val, no changes necessary."
  fi

  log_done "Setting up groups"
  log_done "Clearpath Computer Installer Complete"
  log_info "To continue installation visit: https://docs.clearpathrobotics.com/docs/ros/networking/computer_setup and follow the instructions"
  log_space
else
  log_warn "Clearpath Computer Installer needs to be ran as a user, please re-run."
  log_space
fi

# Re-enable messages about restarting services in systems with needrestart installed
if [ -e /etc/needrestart/conf.d/10-auto-cp.conf ]; then
  sudo rm /etc/needrestart/conf.d/10-auto-cp.conf
fi


if ping -c1 gitlab.clearpathrobotics.com; then
  log_info "Downloading wireless configuration script for use later"
  wget https://gitlab.clearpathrobotics.com/research/lv426-netplan/-/raw/main/configure-lv426.sh -O /home/$USER/setup-lv426.sh
  chmod +x /home/$USER/setup-lv426.sh
fi
