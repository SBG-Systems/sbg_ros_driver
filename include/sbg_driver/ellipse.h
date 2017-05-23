#pragma once

#include "ros/ros.h"

#include "sensor_msgs/Imu.h"
#include "sensor_msgs/NavSatFix.h"
#include "geometry_msgs/PoseStamped.h"
#include "sensor_msgs/TimeReference.h"

#include <sbgEComLib.h>
#include <sbgEComIds.h>

namespace sbg_driver
{
class SbgDriver
{
public:
  SbgDriver(ros::NodeHandle nh);
  virtual ~SbgDriver() {};

  ros::Publisher imu_pub;
  ros::Publisher gps_pub;
  ros::Publisher time_pub;
  ros::Publisher pose_pub;

  void run();

private:
  ros::NodeHandle nh;

  SbgEComHandle       comHandle;
  SbgInterface        sbgInterface;
  SbgEComDeviceInfo   deviceInfo;
  SbgErrorCode        errorCode;
};

 SbgErrorCode onLogReceived(SbgEComHandle *pHandle, SbgEComClass msgClass, SbgEComMsgId msg, const SbgBinaryLogData *pLogData, void *pUserArg);

}
