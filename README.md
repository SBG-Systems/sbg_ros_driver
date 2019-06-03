# sbg_ros_driver
ROS package for SBG Systems IMU.  [SBG Systems](https://www.sbg-systems.com/)

Initial work has been done by [ENSTA Bretagne](https://github.com/ENSTABretagneRobotics).

[sbg_driver Wiki ROS](http://wiki.ros.org/sbg_driver)

SBG ROS driver allows the user to configure the IMU (if possible, according to the device), to receive messages from the Sbg message protocol, some ROS standard messages (ToDo), and to calibrate the magnetometers.

## Installation
### Standard Installation
User can install the sbg_ros_driver through the standard ROS installation system.
* Kinectic
```sudo apt-get install ros-kinetic-sbg-driver```
* Lunar
```sudo apt-get install ros-lunar-sbg-driver```

### Installation from sources
1. Clone the repository (use a Release version)
2. Build using the normal ROS catkin build system
3. Launch the driver using the provided launch file

## Using the Ellipse nodes
### Publishers (`ellipse` node)
  * status ([sbg_driver/SbgStatus](http://docs.ros.org/api/sbg_driver/html/msg/SbgStatus.html))
  * utc_time ([sbg_driver/SbgUtcTime](http://docs.ros.org/api/sbg_driver/html/msg/SbgUtcTime.html))
  * imu_data ([sbg_driver/SbgImuData](http://docs.ros.org/api/sbg_driver/html/msg/SbgImuData.html))
  * ekf_euler ([sbg_driver/SbgEkfEuler](http://docs.ros.org/api/sbg_driver/html/msg/SbgEkfEuler.html))
  * ekf_quat ([sbg_driver/SbgEkfQuat](http://docs.ros.org/api/sbg_driver/html/msg/SbgEkfQuat.html))
  * ekf_nav ([sbg_driver/SbgEkfNav](http://docs.ros.org/api/sbg_driver/html/msg/SbgEkfNav.html))
  * ship_motion ([sbg_driver/SbgShipMotion](http://docs.ros.org/api/sbg_driver/html/msg/SbgShipMotion.html))
  * mag ([sbg_driver/SbgMag](http://docs.ros.org/api/sbg_driver/html/msg/SbgMag.html))
  * mag_calib ([sbg_driver/SbgMagCalib](http://docs.ros.org/api/sbg_driver/html/msg/SbgMagCalib.html))
  * gps_vel ([sbg_driver/SbgGpsVel](http://docs.ros.org/api/sbg_driver/html/msg/SbgGpsVel.html))
  * gps_pos ([sbg_driver/SbgGpsPos](http://docs.ros.org/api/sbg_driver/html/msg/SbgGpsPos.html))
  * gps_hdt ([sbg_driver/SbgGpsHdt](http://docs.ros.org/api/sbg_driver/html/msg/SbgGpsHdt.html))
  * gps_raw ([sbg_driver/SbgGpsRaw](http://docs.ros.org/api/sbg_driver/html/msg/SbgGpsRaw.html))
  * odo_vel ([sbg_driver/SbgOdoVel](http://docs.ros.org/api/sbg_driver/html/msg/SbgOdoVel.html))
  * eventA ([sbg_driver/SbgEvent](http://docs.ros.org/api/sbg_driver/html/msg/SbgEvent.html))
  * eventB ([sbg_driver/SbgEvent](http://docs.ros.org/api/sbg_driver/html/msg/SbgEvent.html))
  * eventC ([sbg_driver/SbgEvent](http://docs.ros.org/api/sbg_driver/html/msg/SbgEvent.html))
  * eventD ([sbg_driver/SbgEvent](http://docs.ros.org/api/sbg_driver/html/msg/SbgEvent.html))
  * pressure ([sbg_driver/SbgPressure](http://docs.ros.org/api/sbg_driver/html/msg/SbgPressure.html))

### Magnetic calibration (`mag_calibration_ellipse` node)
Ellipse-A/E/N use magnemoter to provide heading. A calibration is then required to compensate soft and hard iron distortions due to the environmenent (motors, batteries, ...). The magnetic calibration procedure should be held in a non magnetic area (outside of buildings).
1. Roslaunch the `calibration_sbg_ellipse.launch` 
2. Send a request to the `mag_calibration` service to start the calibration process
3. Proceed to rotations of your IMU (every orientations if possible)
3. Send a request to the `mag_calibration` service to stop the calibration process. An analyse of the calibration is published in log (ROS_INFO)
4. If the calibration is suitable, save the parameters to the Ellipse by sending a request to the `mag_calibration_save` service.

## Q&A
### Enable communication with the ellipse (linux access right)
Ensure that your user is added to the dialout group
To add yourself to the dialout group preform the following :
```sudo adduser username dialout```
  * Where username is your session username
  * Restart your computer to enable access

### Create udev rules
Udev rules can be defined for communication port, in order to avoid modifying the port in configuration if it has changed.
[Udev documentation](https://wiki.debian.org/udev)

A symlink can be configured and defined to uniquely identify the connected device. 

Once it is done, configuration file could be updated `portName: "/dev/sbg"`.

See the docs folder, to see an example of rules with the corresponding screenshot using the udev functions.