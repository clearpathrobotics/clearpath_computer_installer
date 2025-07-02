#!/bin/bash

# This script is only for Humble
ROS_VERSION="humble"

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

# Check if we're running as root; we should not be
current_user="$(whoami)"
if [ "${current_user}" == "root" ];
then
    log_error "This script must not be run as root"
    exit 1
else
    log_info "Running as ${current_user}"
fi

# Check OS version and architecture
# Should be 22.04 and aarch64
ubuntu_release="$(lsb_release -r -s | tail -1)"
if ! [ "${ubuntu_release}" == "22.04" ];
then
  log_error "This script must be run on Ubuntu 22.04. Detected ${ubuntu_release}"
  exit 1
fi
architecture="$(arch)"
if ! [ "${architecture}" == "aarch64" ];
then
  log_error "This script must be run on aarch64. Detected ${architecture}"
  exit 1
fi

# Basic user & group setup
# Make sure we're in the necessary groups for permissions to work properly
log_info "Setting up groups..."
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

if id -nGz "$(whoami)" | grep -qzxF "dialout"; then
  log_info "User:$(whoami) is already in dialout group";
else
  log_info "Adding user:$(whoami) to dialout group";
  sudo usermod -a -G dialout $(whoami);
fi
if id -nGz "$(whoami)" | grep -qzxF "plugdev"; then
  log_info "User:$(whoami) is already in plugdev group";
else
  log_info "Adding user:$(whoami) to plugdev group";
  sudo usermod -a -G plugdev $(whoami);
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

# Enable ROS sources
# https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html
log_info "Enabling ROS software sources..."
sudo apt-get install -y software-properties-common
sudo add-apt-repository universe
sudo apt-get update
sudo apt install -y curl
export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}')
curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo $VERSION_CODENAME)_all.deb" # If using Ubuntu derivates use $UBUNTU_CODENAME
sudo dpkg -i /tmp/ros2-apt-source.deb

# Enable Clearpath sources
# https://packages.clearpathrobotics.com
log_info "Enabling Clearpath software sources..."
wget https://packages.clearpathrobotics.com/public.key -O - | sudo apt-key add -
sudo sh -c 'echo "deb https://packages.clearpathrobotics.com/stable/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/clearpath-latest.list'

# Install initial dependencies
log_info "Installing core ROS components and build tools..."
sudo apt-get update
sudo apt-get install -y \
    jq \
    netplan.io \
    python3-pip \
    python3-rosdep \
    python3-vcstool \
    python3-colcon-common-extensions \
    ros-dev-tools \
    ros-$ROS_VERSION-ros-base

# We'll use yq to make some edits to robot.yaml later
pip3 install yq
export PATH=$PATH:$HOME/.local/bin
grep -qxF 'export PATH=$PATH:$HOME/.local/bin' $HOME/.bashrc || echo 'export PATH=$PATH:$HOME/.local/bin' >> $HOME/.bashrc

if [ -f /opt/ros/$ROS_VERSION/setup.bash ];
then
  source /opt/ros/$ROS_VERSION/setup.bash
else
  log_error "/opt/ros/$ROS_VERSION/setup.bash does not exist. Did ros_base install correctly?"
  exit 1
fi

# Rosdep
log_info "Setting up rosdep..."
sudo rosdep init
sudo wget https://raw.githubusercontent.com/clearpathrobotics/public-rosdistro/master/rosdep/50-clearpath.list -O /etc/ros/rosdep/sources.list.d/50-clearpath.list

# Workspace
log_info "Creating workspace for source installation..."
mkdir -p $HOME/colcon_ws/src
cd $HOME/colcon_ws/src

# Clone & patch packages
log_info "Downloading source packages..."
wget https://raw.githubusercontent.com/clearpathrobotics/clearpath_computer_installer/refs/heads/feature/humble-jetson/jetson-humble.repos
vcs import --input jetson-humble.repos
log_info "Patching ros_kortex for ${architecture}..."
cd ros2_kortex
wget https://raw.githubusercontent.com/clearpathrobotics/clearpath_computer_installer/refs/heads/feature/humble-jetson/ros_kortex_arm.patch
patch -p1 < ros_kortex_arm.patch
cd ..

# Install dependencies
log_info "Installing dependencies with rosdep..."
log_info "  Some dependencies are not available for ${architecture}. Don't panic."
rosdep install --from-paths . --ignore-src -r -y

# Build workspace
log_info "Building the workspace..."
log_info "  This can (and probably will) take a while."
log_info "  Go make yourself a warm beverage and check back later."
cd $HOME/colcon_ws

# Network setup
log_info "Setting up netplan configuration..."
cd $HOME
wget https://raw.githubusercontent.com/clearpathrobotics/clearpath_computer_installer/refs/heads/feature/humble-jetson/50-clearpath-bridge.yaml
sudo mv 50-clearpath-bridge.yaml /etc/netplan
if ping -c 1 gitlab.clearpathrobotics.com &> /dev/null;
then
    log_info "Downloading wireless configuration script for use later..."
    wget https://gitlab.clearpathrobotics.com/research/lv426-netplan/-/raw/main/configure-lv426.sh -O $HOME/setup-lv426.sh
    chmod +x $HOME/setup-lv426.sh
fi

# Robot platform
# Note: for Jetson we only support Jackal, Ridgeback, and Dingo
# available robots; pre-load the user-choice with -1 to indicate undefined
ROBOT_JACKAL_J100=1
ROBOT_RIDGEBACK_R100=2
ROBOT_DINGO_DD100=3
ROBOT_DINGO_DD150=4
ROBOT_DINGO_DO100=5
ROBOT_DINGO_DO150=6
ROBOT_CHOICE=-1

# Get the platform model from the user
step_get_platform_model() {
  if [[ $ROBOT_CHOICE -eq -1 ]];
  then
    echo ""
    prompt_option ROBOT_CHOICE "Which robot platform are you installing?" "Clearpath Jackal J100" "Clearpath Ridgeback R100" "Clearpath Dingo-D DD100" "Clearpath Dingo-D DD150" "Clearpath Dingo-O DO100" "Clearpath Dingo-O DO150"
  fi
  case "$ROBOT_CHOICE" in
    $ROBOT_JACKAL_J100)
      platform="j100"
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

# Clearpath robot.yaml setup
if [ -e /etc/clearpath/robot.yaml ]; then
  log_warn "Clearpath Robot YAML exists"
  prompt_YESno update_config "Would you like to change Clearpath Robot YAML?"
  if [[ $update_config == "y" ]]; then
    sudo mv /etc/clearpath/robot.yaml /etc/clearpath/robot.yaml.backup.$(date +"%Y%m%d%H%M%S")
    log_info "Creating default robot YAML for ${platform}"
    sudo cp /opt/ros/$ROS_VERSION/share/clearpath_config/sample/${platform}_default.yaml /etc/clearpath/robot.yaml
    # Check if sources were added
    if [ ! -e /etc/clearpath/robot.yaml ]; then
      log_error "Failed to create Clearpath robot YAML"
      exit 1
    fi
  else
    log_info "No change to Clearpath Robot YAML"
  fi
else
  log_info "Creating default robot YAML for ${platform}"
  cp /opt/ros/$ROS_VERSION/share/clearpath_config/sample/${platform}_default.yaml /etc/clearpath/robot.yaml
  # Check if sources were added
  if [ ! -e /etc/clearpath/robot.yaml ]; then
    log_error "Failed to create Clearpath robot YAML"
    exit 1
  fi
fi

while true; do
  log_info "Please enter the serial number of the robot (Only 4 digits, the platform model will be automatically added):"
  read serial_number
  # Regular expression to match 4 numbers
  pattern='^[0-9]{4}$'

  if [[ $serial_number =~ $pattern ]]; then
    log_info "Serial number is in the correct format."
    break
  else
    log_error "Serial number is not in the correct format. It should consist of exactly 4 numbers."
  fi
done

platform_serial_number="$platform-$serial_number"
platform_namespace="$platform"_"$serial_number"

# Set the serial number and namespace
yq -i -y ".serial_number = \"${platform_serial_number}\"" /etc/clearpath/robot.yaml
yq -i -y ".system.ros2.namespace = \"${platform_namespace}\"" /etc/clearpath/robot.yaml

# Set the hostname for the default host
yq -i -y ".system.hosts[0].hostname = \"$(hostname)\"" /etc/clearpath/robot.yaml

# Add the workspace we created & built above
yq -i -y ".system.ros2.workspaces = [\"$HOME/colcon_ws/install/setup.bash\"]" /etc/clearpath/robot.yaml

# Install systemd jobs
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

log_info "Setting up Clearpath environment..."
grep -qxF "source /etc/clearpath/setup.bash" $HOME/.bashrc || echo "source /etc/clearpath/setup.bash" >> $HOME/.bashrc

log_done "Done setting up Jetson with ROS 2 Humble. Please reboot now"
