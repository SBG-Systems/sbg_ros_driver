# sbg_driver
## Overview
ROS package for SBG Systems IMU.<br />
The driver allows the user to configure the IMU (if possible, according to the device), to receive messages from the Sbg message protocol, publish ROS standard messages , and to calibrate the magnetometers.

<i>Initial work has been done by [ENSTA Bretagne](https://github.com/ENSTABretagneRobotics).</i>

**Author : [SBG Systems](https://www.sbg-systems.com/)<br />
Maintainer : Erwan Cabon, erwan.cabon@sbg-systems.com**

## Installation
### Installation from Packages
User can install the sbg_ros_driver through the standard ROS installation system.
* Melodic ```sudo apt-get install ros-melodic-sbg-driver```
* Kinectic ```sudo apt-get install ros-kinetic-sbg-driver```
* Lunar ```sudo apt-get install ros-lunar-sbg-driver```

### Building from sources
#### Dependencies
* [Robot Operating System (ROS)](http://wiki.ros.org/)
* SBG communication protocol sbgECom, v1.11.920-stable (full compatible with firmwares from 1.7.x).

#### Building
1. Clone the repository (use a Release version)
2. Build using the normal ROS catkin build system

```
cd catkin_ws/src
git clone https://github.com/SBG-Systems/sbg_ros_driver.git
cd ../
catkin_make
```

## Usage
To run the default Ros node with the default configuration

```
roslaunch sbg_driver sbg_device.launch
```

To run the magnetic calibration node

```
roslaunch sbg_driver sbg_device_mag_calibration.launch
```

## Config files
### Default config files
Every configuration file is defined according to the same structure.<br />

* **sbg_device_uart_default.yaml** <br />
This config file is the default one for Uart connection with the device. <br />
It does not configure the device through the ROS node, so it has to be previously configured (manually or with the ROS node). <br />
It defines a few outputs for the device :
	* `/sbg/imu_data`, `/sbg/ekf_quat` at 25Hz
	* ROS standard outputs `/imu/data`, `/imu/velocity`, `/imu/temp` at 25Hz
	* `/sbg/status`, `/sbg/utc_time` and `/imu/utc_ref` at 1Hz.

* **sbg_device_udp_default.yaml** <br />
This config file is the default one for an Udp connection with the device. <br />
It does not configure the device through the ROS node, so it has to be previously configured (manually or with the ROS node). <br />
It defines a few outputs for the device : 
	* `/sbg/imu_data`, `/sbg/ekf_quat` at 25Hz
	* ROS standard outputs `/imu/data`, `/imu/velocity`, `/imu/temp` at 25Hz
	* `/sbg/status`, `/sbg/utc_time` and `/imu/utc_ref` at 1Hz.

### Example config files
* **ellipse_A_default.yaml** <br />
Default config file for an Ellipse-A.

* **ellipse_E_default.yaml** <br />
Default config file for an Ellipse-E with an external antenna and external Gnss.

* **ellipse_N_default.yaml** <br />
Default config file for an Ellipse-N with an external antenna and internal Gnss.

## Launch files
### Default launch files
* **sbg_device.launch** <br />
Launch the sbg_device node to handle the receivde data, and load the `sbg_device_uart_default.yaml` configuration.

* **sbg_device_mag_calibration.launch** <br />
Launch the sbg_device_mag node to calibrate the magnetometers, and load the `ellipse_E_default.yaml` configuration.

## Nodes
### sbg_device
The sbg_device node handles the communication with the connected device, and publishes the SBG output to the Ros environment.

#### Published Topics
##### SBG specific topics
* **`/sbg/status`** [sbg_driver/SbgStatus](http://docs.ros.org/api/sbg_driver/html/msg/SbgStatus.html)

  Provides informations about the general status (Communication, Aiding, etc..).
  
* **`/sbg/utc_time`** [sbg_driver/SbgUtcTime](http://docs.ros.org/api/sbg_driver/html/msg/SbgUtcTime.html)

  Provides UTC time reference.

* **`/sbg/imu_data`** [sbg_driver/SbgImuData](http://docs.ros.org/api/sbg_driver/html/msg/SbgImuData.html)

  IMU status, and sensors values.
  
* **`/sbg/ekf_euler`** [sbg_driver/SbgEkfEuler](http://docs.ros.org/api/sbg_driver/html/msg/SbgEkfEuler.html)

  Computed orientation using Euler angles.
  
* **`/sbg/ekf_quat`** [sbg_driver/SbgEkfQuat](http://docs.ros.org/api/sbg_driver/html/msg/SbgEkfQuat.html)

  Computed orientation using Quaternion.
  
* **`/sbg/ekf_nav`** [sbg_driver/SbgEkfNav](http://docs.ros.org/api/sbg_driver/html/msg/SbgEkfNav.html)

  Computed navigation data.
  
* **`/sbg/mag`** [sbg_driver/SbgMag](http://docs.ros.org/api/sbg_driver/html/msg/SbgMag.html)

  Magnetic data.
  
* **`/sbg/mag_calib`** [sbg_driver/SbgMagCalib](http://docs.ros.org/api/sbg_driver/html/msg/SbgMagCalib.html)

  Magnetometer calibration data.
  
* **`/sbg/ship_motion`** [sbg_driver/SbgShipMotion](http://docs.ros.org/api/sbg_driver/html/msg/SbgShipMotion.html)

  Heave, surge and sway data.
  
* **`/sbg/gps_vel`** [sbg_driver/SbgGpsVel](http://docs.ros.org/api/sbg_driver/html/msg/SbgGpsVel.html)

  GPS velocities from GPS receiver.
  
* **`/sbg/gps_pos`** [sbg_driver/SbgGpsPos](http://docs.ros.org/api/sbg_driver/html/msg/SbgGpsPos.html)

  GPS positions from GPS receiver.
  
* **`/sbg/gps_hdt`** [sbg_driver/SbgGpsHdt](http://docs.ros.org/api/sbg_driver/html/msg/SbgGpsHdt.html)

  GPS true heading from dual antenna system.
  
* **`/sbg/gps_raw`** [sbg_driver/SbgGpsRaw](http://docs.ros.org/api/sbg_driver/html/msg/SbgGpsRaw.html)

  GPS raw data for post processing.
  
* **`/sbg/odo_vel`** [sbg_driver/SbgOdoVel](http://docs.ros.org/api/sbg_driver/html/msg/SbgOdoVel.html)

  Odometer velocity.
  
* **`/sbg/event[ABCDE]`** [sbg_driver/SbgEvent](http://docs.ros.org/api/sbg_driver/html/msg/SbgEvent.html)

  Event on sync in the corresponding pin.
  
* **`/sbg/pressure`** [sbg_driver/SbgPressure](http://docs.ros.org/api/sbg_driver/html/msg/SbgPressure.html)

  Pressure data.

##### ROS standard topics
In order to define ROS standard topics, it requires sometimes several SBG messages, to be merged.
For each ROS standard, you have to activate the needed SBG outputs.

* **`/imu/data`** [sensor_msgs/Imu](http://docs.ros.org/melodic/api/sensor_msgs/html/msg/Imu.html)

  IMU data.
  Requires `/sbg/imu_data` and `/sbg/ekf_quat`.
  
* **`/imu/temp`** [sensor_msgs/Temperature](http://docs.ros.org/melodic/api/sensor_msgs/html/msg/Temperature.html)

  IMU temperature data.
  Requires `/sbg/imu_data`.
  
* **`/imu/velocity`** [geometry_msgs/TwistStamped](http://docs.ros.org/melodic/api/geometry_msgs/html/msg/TwistStamped.html)

  IMU velocity data.
  Requires `/sbg/imu_data`.
  
* **`/imu/mag`** [sensor_msgs/MagneticField](http://docs.ros.org/melodic/api/sensor_msgs/html/msg/MagneticField.html)

  IMU magnetic field.
  Requires `/sbg/mag`.
  
* **`/imu/pres`** [sensor_msgs/FluidPressure](http://docs.ros.org/melodic/api/sensor_msgs/html/msg/FluidPressure.html)

  IMU pressure data.
  Requires `/sbg/pressure`.
  
* **`/imu/pos_ecef`** [geometry_msgs/PointStamped](http://docs.ros.org/melodic/api/geometry_msgs/html/msg/PointStamped.html)

  Earth-Centered Earth-Fixed position.
  Requires `/sbg/ekf_nav`.
  
* **`/imu/utc_ref`** [sensor_msgs/TimeReference](http://docs.ros.org/melodic/api/sensor_msgs/html/msg/TimeReference.html)

  UTC time reference.
  Requires `/sbg/utc_time`.
  
* **`/imu/nav_sat_fix`** [sensor_msgs/NavSatFix](http://docs.ros.org/melodic/api/sensor_msgs/html/msg/NavSatFix.html)

  Navigation satellite fix for any Global Navigation Satellite System.
  Requires `/sbg/gps_pos`.
  

### sbg_device_mag
The sbg_device_mag node handles the magnetic calibration for suitable devices.

#### Services
* **`/sbg/mag_calibration`** [std_srvs/Trigger](http://docs.ros.org/api/std_srvs/html/srv/Trigger.html)

  Service to start/stop the magnetic calibration.

* **`/sbg/mag_calibration_save`** [std_srvs/Trigger](http://docs.ros.org/api/std_srvs/html/srv/Trigger.html)

  Service to save the magnetic calibration to the connected device.

## HowTo
### Configure the SBG device
The SBG Ros driver allows the user to configure the device before starting the data handling. <br />
To do so, set the corresponding parameter in the used config file.

```
# Configuration of the device with ROS.
confWithRos: true
```

Then, modify the desired parameters in the config file, using the <i>SBG Firmware Manual</i>, to see which features are configurable, and which parameter values are available.

### Calibrate the magnetometers
Ellipse-A/E/N use magnemoter to provide heading. A calibration is then required to compensate soft and hard iron distortions due to the environmenent (motors, batteries, ...). The magnetic calibration procedure should be held in a non magnetic area (outside of buildings).

```
roslaunch sbg_driver sbg_device_mag_calibration.launch
rosservice call /sbg/mag_calibration
```

> success: True<br />
> message: "Magnetometer calibration process started."

Proceed rotations of the IMU (every orientation if possible).

```
rosservice call /sbg/mag_calibration
```

> success: True<br />
> message: "Magnetometer calibration is finished. See the output console to get calibration informations."

If the magnetic calibration is satisfaying (Quality, Confidence), it could be uploaded/saved to the device.

```
rosservice call /sbg/mag_calibration_save
```

> success: True<br />
> message: "Magnetometer calibration has been uploaded to the device."

### Enable communication with the SBG device
To be able to communicate with the device, be sure that your user is part of the dialout group.<br />
Once added, restart your machine to save and apply the changes.

```
sudo adduser $USER dialout
```

### Create udev rules
Udev rules can be defined for communication port, in order to avoid modifying the port in configuration if it has changed.
[Udev documentation](https://wiki.debian.org/udev)

A symlink can be configured and defined to uniquely identify the connected device. <br />
Once it is done, configuration file could be updated `portName: "/dev/sbg"`.

See the docs folder, to see an example of rules with the corresponding screenshot using the udev functions.

### Synchronize the ROS messages from an external time source
#### No external source
When no external time source is available, the header time of messages is the system Epoch (Unix) processing time of the SBG callback, given by the ```ros::Time::now()```.

#### External Gnss receiver and/or External antenna
When the SBG device is connected with an external Gnss receiver, if the device receives a full valid SBG Utc log, i.e :
* A stable input clock to be synchronized with the internal clock
* A valid Utc time data (with or without the leap second)
* The clock has converged to the PPS

then, the time header will be computed from the last received Utc log and the device timestamp given by the internal clock. 

## Contributing
### Bugs and issues
Please report bugs and/or issues using the [Issue Tracker](https://github.com/SBG-Systems/sbg_ros_driver/issues)

### Features requests or additions
In order to contribute to the code, please use Pull requests to the `devel` branch.<br />
If you have some feature requests, use the [Issue Tracker](https://github.com/SBG-Systems/sbg_ros_driver/issues) as well.