# clearpath_computer_installer

Contains scripts to set up a Clearpath robot computer for ROS 2 (Humble or Jazzy). For full instructions visit https://docs.clearpathrobotics.com/docs/ros/installation/robot.

This script includes setting of ROS 2 and Clearpath package servers, installing ROS 2 and Clearpath Robot packages and installing the Clearpath Robot Service.

## Usage - Standard `amd64` computer
```
wget -c https://raw.githubusercontent.com/clearpathrobotics/clearpath_computer_installer/main/clearpath_computer_installer.sh && bash -e clearpath_computer_installer.sh
```

## Usage - Nvidia Jetson computer
**Note** Only ROS 2 Humble is currently usable on Nvidia Jetson computers. Only Ridgeback, Dingo-O, Dingo-D, and Jackal can be configured to use an Nvidia Jetson as their primary computer.

Install JetPack 6.2.x (6.2.1 is the latest at the time of writing) per the manufacturer's instructions. Then connect the Jetson to the internet and run the following command:
```
wget -c https://raw.githubusercontent.com/clearpathrobotics/clearpath_computer_installer/refs/heads/jetson-setup/jetson-humble-setup.bash && bash -e jetson-humble-setup.bash
```
This script has been tested with the Nvidia Orin NX on a [Forecr ORNX](https://www.forecr.io/products/carrier-board-dsboard-ornx) carrier board. For step-by-step instructions on installing JetPack on this setup, see [Forecr's instructions](https://www.forecr.io/blogs/installation/jetpack-6-x-installation-for-dsboard-ornx).

### M.2 Wireless/Bluetooth support on ORNX boards
By default the ORNX carrier board does not have a USB connection to the M.2 Key-E socket, in which the wireless+bluetooth card is normally installed. To enable bluetooth support the following commands must be run:
```
sudo echo 352 > /sys/class/gpio/export
sudo echo high > /sys/class/gpio/PA.04/direction
```
These commands can be added to `/etc/rc.local` by the `jetson-humble-setup.bash` script by answering `Y` when prompted. These commands _disconnect_ one of the carrier board's USB-A sockets and redirects the USB connection to the M.2 Key-E socket. This means that only one USB port will be usable and the other is not. Be careful when connecting USB devices and ensure only the _connected_ USB port is used.
