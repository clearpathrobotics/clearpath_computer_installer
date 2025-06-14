#!/bin/bash -e
# Software License Agreement (BSD)
#
# Author    Hilary Luo <hluo@clearpathrobotics.com>
# Copyright (c) 2025, Clearpath Robotics, Inc., a Rockwell Automation Company. All rights reserved.
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

log_info "Installing cockpit"

step_setup_cpr_packge_server
sudo apt -y -qq update

sudo apt -y -qq install -t noble-backports cockpit
sudo apt -y -qq install ros-${ROS_DISTRO}-foxglove-bridge cockpit-ros2-diagnostics

# Config file name
file_name="/etc/cockpit/cockpit.conf"
file_contents="[WebService]
AllowUnencrypted=true"

prompt_YESno allow_unencrypted "Allow unencrypted connections for cockpit? Required for ROS connectivity."
if [[ $allow_unencrypted == "y" ]]; then
  # Check if the file exists
  if [ -f "$file_name" ]; then
    if ! grep -q "AllowUnencrypted=true" $file_name; then
      log_warn "$file_name file already exists but does not contain the WebService AllowUnencrypted=true line. Must be manually verified / added."
    else
      log_info "$file_name file already exists and contains the AllowUnencrypted=true line. No action needed."
    fi
  else
    echo "$file_contents" | sudo tee $file_name > /dev/null
  fi
else
  log_error "Skipping unencrypted connection for cockpit. ROS connectivity requires this so either it must be set up manually, cockpit only accessed from localhost or cockpit put behind a reverse proxy."
fi

# Disable the software update tab in cockpit because it does not work with networkd and can cause confusion.
override_file_name="/etc/cockpit/packagekit.override.json"
override_file_contents='{
  "tools": {
    "index": null
  }
}'

prompt_YESno disable_update_tab "Hide software update tab in cockpit? This is recommended because it does not work with networkd and can cause confusion. Run updates in the terminal instead."
if [[ $disable_update_tab == "y" ]]; then
  # Check if the file exists
  if [ -f "$override_file_name" ]; then
    if ! grep -q '"index": null' $override_file_name; then
      log_warn "$override_file_name file already exists but does not contain the \"index\": null line. Must be manually verified / added."
    else
      log_info "$override_file_name file already exists and contains the \"index\": null line. No action needed."
    fi
  else
    echo "$override_file_contents" | sudo tee $override_file_name > /dev/null
  fi
else
  log_error "Skipping hiding the software update tab for cockpit. Be aware that this tab may not work properly for Clearpath robots and may be misleading."
fi

log_done "Installed cockpit"
