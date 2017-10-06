# sbg_driver
ROS driver for SBG System Ellipse IMU

## Standard Installation

sudo apt-get install ros-kinetic-sbg-driver
(You can change the version of ros "kinectic" -> "x")

## Installation from sources

1. Clone the repository (use a Release version)
2. Build using the normal ROS catkin build system
3. Ensure that your user is added to the dialout group
  * To add yourself to the dialout group preform the following
  * sudo adduser username dialout
  * Where username is your session username
  * Restart your computer to enable access
4. Launch the driver using the provided launch file
