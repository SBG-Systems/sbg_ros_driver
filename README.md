# sbg_driver
ROS driver package for SBG Systems IMU, AHRS and INS.  
This driver package uses the [sbgECom binary protocol](https://github.com/SBG-Systems/sbgECom) to read data and configure SBG Systems devices.  

<i>Initial work has been done by [ENSTA Bretagne](https://github.com/ENSTABretagneRobotics).</i>

**Author: [SBG Systems](https://www.sbg-systems.com/)**  
**Maintainer: [SBG Systems](https://www.sbg-systems.com/)**  
**Contact:** support@sbg-systems.com

## Features
The driver supports the following features:
 - Configure ELLIPSE products using yaml files (see note below)
 - Parse IMU/AHRS/INS/GNSS using the sbgECom protocol
 - Publish standard ROS messages and more detailed specific SBG Systems topics
 - Subscribe and forward RTCM data to support DGPS/RTK mode with centimeters-level accuracy
 - Calibrate 2D/3D magnetic field using the on-board ELLIPSE algorithms

> [!NOTE]
> Only ELLIPSE devices can be configured from the ROS driver. For High Performance INS such as EKINOX, APOGEE and QUANTA, please use the [sbgInsRestApi](https://developer.sbg-systems.com/sbgInsRestApi/)

## Installation
### Installation from Packages
User can install the sbg_ros_driver through the standard ROS installation system.

* Melodic ```sudo apt-get install ros-melodic-sbg-driver```
* Noetic ```sudo apt-get install ros-noetic-sbg-driver```

### Building from sources
#### Dependencies
* [Robot Operating System (ROS)](http://wiki.ros.org/)
* [sbgECom C Library](https://github.com/SBG-Systems/sbgECom) (embeds v1.11.920-stable - compatible with ELLIPSE firmware above 1.7)
* [Boost C++ Library](https://www.boost.org/)

#### Building
1. Clone the repository (use a Release version)
```
cd catkin_ws/src
git clone -b master https://github.com/SBG-Systems/sbg_ros_driver.git
```

2. Install dependencies
```
cd ..
rosdep install --from-paths src --ignore-src -r -y
```

3. Build using the normal ROS catkin build system

```
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
Every configuration file is defined according to the same structure.  

* **sbg_device_uart_default.yaml**
This config file is the default one for UART connection with the device.  
It does not configure the device through the ROS node, so it has to be previously configured (manually or with the ROS node).  
It defines a few outputs for the device:
  * `/sbg/imu_data`, `/sbg/ekf_quat` at 25Hz
  * ROS standard outputs `/imu/data`, `/imu/velocity`, `/imu/temp` at 25Hz
  * `/sbg/status`, `/sbg/utc_time` and `/imu/utc_ref` at 1Hz.

* **sbg_device_udp_default.yaml**
This config file is the default one for an Udp connection with the device.  
It does not configure the device through the ROS node, so it has to be previously configured (manually or with the ROS node).  
It defines a few outputs for the device:
  * `/sbg/imu_data`, `/sbg/ekf_quat` at 25Hz
  * ROS standard outputs `/imu/data`, `/imu/velocity`, `/imu/temp` at 25Hz
  * `/sbg/status`, `/sbg/utc_time` and `/imu/utc_ref` at 1Hz.

### Example config files
* **ellipse_A_default.yaml**
Default config file for an Ellipse-A.

* **ellipse_E_default.yaml**
Default config file for an Ellipse-E with an external NMEA GNSS.

* **ellipse_N_default.yaml**
Default config file for an Ellipse-N using internal GNSS.

* **ellipse_D_default.yaml**
Default config file for an Ellipse-D using internal GNSS.

## Launch files
### Default launch files
* **sbg_device_launch.py**
Launch the sbg_device node to handle the received data, and load the `sbg_device_uart_default.yaml` configuration.

* **sbg_device_mag_calibration_launch.py**
Launch the sbg_device_mag node to calibrate the magnetometers, and load the `ellipse_E_default.yaml` configuration.

## Nodes
### sbg_device node
The `sbg_device` node handles the communication with the connected device, publishes the SBG output to the Ros environment and subscribes to useful topics such as RTCM data streams.

#### Published Topics
##### SBG Systems specific topics
SBG Systems has defined proprietary ROS messages to report more detailed information from the AHRS/INS.  
These messages try to match as much as possible the sbgECom logs as they are output by the device.

* **`/sbg/status`** [sbg_driver/SbgStatus](http://docs.ros.org/api/sbg_driver/html/msg/SbgStatus.html)

  Provides information about the general status (Communication, Aiding, etc..).
  
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

  Calibrated magnetic field measurement.
  
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
  
* **`/imu/odometry`** [nav_msgs/Odometry](http://docs.ros.org/en/melodic/api/nav_msgs/html/msg/Odometry.html)

  UTM projected position relative to the first valid INS position.
  Requires `/sbg/imu_data` and `/sbg/ekv_nav` and either `/sbg/ekf_euler` or `/sbg/ekf_quat`.
  Disabled by default, set `odometry.enable` in configuration file.

> [!NOTE]
> Please update the driver configuration to enable standard ROS messages publication. Also, the driver only publish standard ROS messages if the driver is setup to use ENU frame convention.

##### NMEA topics
The driver can publish NMEA GGA messages from the internal GNSS receiver. It can be used with third party [NTRIP client](https://github.com/LORD-MicroStrain/ntrip_client) modules to support VRS networks providers.

 Disabled by default, set `nmea.publish` to `true` in .yaml config file to use this feature.

* **`/ntrip_client/nmea`** [nmea_msgs/Sentence](http://docs.ros.org/en/api/nmea_msgs/html/msg/Sentence.html)  
  
  Data from `/sbg/gps_pos` serialized into NMEA GGA format. Requires `/sbg/gps_pos`.  
  Namespace `ntrip_client` and topic_name `nmea` can be customized in .yaml config files.

#### Subscribed Topics
##### RTCM topics
The `sbg_device` node can subscribe to RTCM topics published by third party ROS2 modules.  
Incoming RTCM data are forwarded to the INS internal GNSS receiver to enable DGPS/RTK solutions.

 Disabled by default, set `rtcm.subscribe` to `true` in .yaml config file to use this feature.

* **`/ntrip_client/rtcm`** [rtcm_msgs/Message](https://github.com/tilk/rtcm_msgs/blob/master/msg/Message.msg)

  RTCM data from `/ntrip_client/rtcm` will be forwarded to the internal INS GNSS receiver.  
  Namespace `ntrip_client` and topic_name `rtcm` can be customized in .yaml config files.

### sbg_device_mag node
The sbg_device_mag node is used to execute on board in-situ 2D or 3D magnetic field calibration.  
If you are planning to use magnetic based heading, it is mandatory to perform a magnetic field calibration in a clean magnetic environnement.

Only ELLIPSE products support magnetic based heading and feature the on-board magnetic field calibration process.

#### Services
* **`/sbg/mag_calibration`** [std_srvs/Trigger](http://docs.ros.org/api/std_srvs/html/srv/Trigger.html)

  Service to start/stop the magnetic calibration.

* **`/sbg/mag_calibration_save`** [std_srvs/Trigger](http://docs.ros.org/api/std_srvs/html/srv/Trigger.html)

  Service to save in FLASH memory the latest computed magnetic field calibration.

## HowTo
### Configure the SBG device
The SBG Ros driver allows the user to configure the device before starting data parsing.  
To do so, set the corresponding parameter in the used config file.

```
# Configuration of the device with ROS.
confWithRos: true
```

Then, modify the desired parameters in the config file, using the [Firmware Reference Manual](https://support.sbg-systems.com/sc/dev/latest/firmware-documentation), to see which features are configurable, and which parameter values are available.

### Configure for RTK/DGPS
The `sbg_device` node can subscribe to [rtcm_msgs/Message](https://github.com/tilk/rtcm_msgs/blob/master/msg/Message.msg) topics to forward differential corrections to the INS internal GNSS receiver.

The RTCM data stream is sent through the serial/ethernet interface used by ROS to communicate with the INS.  
This enables simple and efficient RTK operations without requiring additional hardware or wiring.

When combined with a third party [NTRIP client](https://github.com/LORD-MicroStrain/ntrip_client), it offers a turnkey solution to access local VRS providers and get centimeter-level accuracy solutions.

The driver and the device should be properly setup:
 - Configure the INS to accept RTCM corrections on the interface used by the ROS driver:
   - For ELLIPSE, simply use the `sbgCenter` and in `Assignment panel`, `RTCM` should be set to `Port A`.
   - For High Performance INS, either use the configuration web interface or the [sbgInsRestApi](https://developer.sbg-systems.com/sbgInsRestApi/).
 - Install and configure a third party node that broadcast RTCM corrections such as a [NTRIP client](https://github.com/LORD-MicroStrain/ntrip_client)
 - Update the node config `yaml` file to set `rtcm.subscribe` and `nmea.publish` to `true`
 - If you use a different node to broadcast RTCM topics, you might have to update the config `yaml` file to update topics and namespaces.

### Calibrate the magnetometers
ELLIPSE products can use magnetometers to determine the heading. A calibration is then required to compensate for soft and hard iron distortions due to the vehicle the product is installed on. The magnetic calibration procedure should be held in a clean magnetic environnement (outside of buildings).

You can read more information about magnetic field calibration procedure from the SBG Systems [Support Center](https://support.sbg-systems.com/sc/kb/v3/inertial-sensors-installation/magnetic-calibration).

The ROS driver provides a dedicated node to easily use ELLIPSE on board magnetic field calibration algorithms.  
The ELLIPSE offers both a 2D and 3D magnetic field calibration mode.

1) Make sure you have selected the desired 2D or 3D magnetic field calibration mode (`calibration.mode` in the configuration `yaml` file).
2) Start a new magnetic calibration session once you are ready to map the magnetic field:

```
roslaunch sbg_driver sbg_device_mag_calibration.launch
rosservice call /sbg/mag_calibration
```

> success: True  
> message: "Magnetometer calibration process started."

3) Rotate as much as possible the unit to map the surrounding magnetic field (ideally, perform a 360° with X then Y then Z axis pointing downward).
4) Once you believe you have covered enough orientations, compute a magnetic field calibration:

```
rosservice call /sbg/mag_calibration
```

> success: True  
> message: "Magnetometer calibration is finished. See the output console to get calibration information."

5) If you are happy with the results (Quality, Confidence), apply and save the new magnetic calibration parameters.  
   If not, you can continue to rotate the product and try to perform a new computation (and repeat step 4)

```
rosservice call /sbg/mag_calibration_save
```

> success: True  
> message: "Magnetometer calibration has been uploaded to the device."

6) Reset/Power Cycle the device and you should now get an accurate magnetic based heading.

### Enable communication with the SBG device
To be able to communicate with the device, be sure that your user is part of the dialout group.  
Once added, restart your machine to save and apply the changes.

```
sudo adduser $USER dialout
```

### Create udev rules
Udev rules can be defined for communication port, in order to avoid modifying the port in configuration if it has changed.
[Udev documentation](https://wiki.debian.org/udev)

A symlink can be configured and defined to uniquely identify the connected device.  
Once it is done, configuration file could be updated `portName: "/dev/sbg"`.

See the docs folder, to see an example of rules with the corresponding screenshot using the udev functions.

### Time source & reference
ROS uses an internal system time to time stamp messages. This time stamp is generally gathered when the message is processed and published.
As a result, the message is not time stamped accurately due to transmission and processing delays.

SBG Systems INS however provides a very accurate timing based on GNSS time if available. The following conditions have to be met to get
absolute accurate timing information:
* The ELLIPSE-N or D should have a connected GNSS antenna with internal GNSS enabled
* The ELLIPSE-E should be connected to an external GNSS receiver with a PPS signal
* A valid GNSS position has to be available to get UTC data
* The ELLIPSE internal clock should be aligned to PPS signal (clock status)
* The ELLIPSE should be setup to send SBG_ECOM_LOG_UTC message

You can select which time source to use with the parameter `time_reference` to time stamp messages published by this driver:
* `ros`: The header.stamp member contains the current ROS system time when the message has been processed.
* `ins_unix`: The header.stamp member contains an absolute and accurate time referenced to UNIX epoch (00:00:00 UTC on 1 January 1970)

Configuration example to use an absolute and accurate time reference to UNIX epoch:
```
# Time reference:
time_reference: "ins_unix"
```

## Frame parameters & conventions
### Frame ID
The frame_id of the header can be set with this parameter:
```
# Frame name
frame_id: "imu_link_ned"
```

### Frame convention
The frame convention can be set to NED or ENU:
* The NED convention is SBG Systems native convention so no transformation is applied
* The ENU convention follows ROS standard [REP-103](https://www.ros.org/reps/rep-0103.html#coordinate-frame-conventions)

Please read the SBG Systems [Support Center article](https://support.sbg-systems.com/sc/kb/latest/underlying-maths-conventions) for more details.

You can select the frame convention to use with the following parameter:
```
# Frame convention
use_enu: true
```

> [!NOTE]
> The driver only publish standard ROS messages if the driver is setup to use ENU frame convention.

#### Body/Vehicle Frame:
The X axis should point the vehicle **forward** direction for both NED and ENU frame conventions. 
The table below summarizes the body/vehicle axis frame definitions for each convention:

| NED Convention | ENU Convention |
| -------------- | -------------- |
| X Forward      | X Forward      |
| Y Right        | Y Left         |
| Z Downward     | Z Upward       |

#### Navigation Frame:

The navigation frame also referred by ROS as the cartesian representation is defined as follow:

| NED Convention | ENU Convention |
| -------------- | -------------- |
| X North        | X East         |
| Y East         | Y North        |
| Z Down         | Z Up           |

#### Heading Example:

Based on the definitions above, when using a NED frame, if the vehicle X axis is pointing North, the INS should return a zero heading. 
When using a ENU frame, the INS should return a zero heading when the vehicle X axis is pointing East.

## Troubleshooting

If you experience higher latency than expected and have connected the IMU via an USB interface, you can enable the serial driver low latency mode:
```
/bin/setserial /dev/<device> low_latency
```

## Contributing
### Bugs and issues
Please report bugs and/or issues using the [Issue Tracker](https://github.com/SBG-Systems/sbg_ros_driver/issues)

### Features requests or additions
In order to contribute to the code, please use Pull requests to the `devel` branch.  
If you have some feature requests, use the [Issue Tracker](https://github.com/SBG-Systems/sbg_ros_driver/issues) as well.
