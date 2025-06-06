# clearpath_computer_installer

Contains a script to set up a Clearpath robot computer for ROS 2 Humble or Jazzy on Ubuntu. For full instructions visit https://docs.clearpathrobotics.com/docs/ros/installation/robot.

This full script includes setting of ROS 2 and Clearpath package servers, installing ROS 2 and Clearpath Robot packages and installing the Clearpath Robot Service.

Additional Scripts that are run as part of the main script but can also be run independently:
`cockpit_installer.sh` - To install cockpit and the ros2 plugin (available for Noble / Jazzy only)

## Usage

Full install:
```
wget -c https://raw.githubusercontent.com/clearpathrobotics/clearpath_computer_installer/main/clearpath_computer_installer.sh && bash -e clearpath_computer_installer.sh
```

Cockpit only install (available for Noble / Jazzy only):
```
wget -c https://raw.githubusercontent.com/clearpathrobotics/clearpath_computer_installer/main/cockpit_installer.sh && bash -e cockpit_installer.sh
```