#include "sbg_driver/ellipse.h"

#include <cmath>
#include "boost/date_time/posix_time/posix_time_types.hpp"

#include <diagnostic_updater/diagnostic_updater.h>

using namespace sbg_driver;

using namespace boost::posix_time;
using namespace boost::gregorian;

/*!
 *  Callback definition called each time a new log is received.
 *  \param[in]  pHandle                 Valid handle on the sbgECom instance that has called this callback.
 *  \param[in]  msgClass                Class of the message we have received
 *  \param[in]  msg                   Message ID of the log received.
 *  \param[in]  pLogData                Contains the received log data as an union.
 *  \param[in]  pUserArg                Optional user supplied argument.
 *  \return                       SBG_NO_ERROR if the received log has been used successfully.
 */
SbgErrorCode sbg_driver::onLogReceived(SbgEComHandle *pHandle, SbgEComClass msgClass, SbgEComMsgId msg, const SbgBinaryLogData *pLogData, void *pUserArg)
{
  SbgDriver* node = static_cast<SbgDriver*>(pUserArg);
  ROS_DEBUG_NAMED("sbgECom", "received log data id: %d", msg);

  // float time_of_week;
  switch (msg){
    case SBG_ECOM_LOG_EKF_QUAT:
    {
      sensor_msgs::Imu imu_msg;
      geometry_msgs::PoseStamped pose_msg;

      imu_msg.header.stamp = ros::Time::now();
      imu_msg.header.frame_id = "map";
      pose_msg.header.stamp = ros::Time::now();
      pose_msg.header.frame_id = "map";

      imu_msg.orientation.x = pLogData->ekfEulerData.euler[1];
      imu_msg.orientation.y = pLogData->ekfEulerData.euler[2];
      imu_msg.orientation.z = pLogData->ekfEulerData.euler[3];
      imu_msg.orientation.w = pLogData->ekfEulerData.euler[0];

      pose_msg.pose.orientation.x = pLogData->ekfEulerData.euler[1];
      pose_msg.pose.orientation.y = pLogData->ekfEulerData.euler[2];
      pose_msg.pose.orientation.z = pLogData->ekfEulerData.euler[3];
      pose_msg.pose.orientation.w = pLogData->ekfEulerData.euler[0];

      node->imu_pub.publish(imu_msg);
      node->pose_pub.publish(pose_msg);
      break;
    }

    case SBG_ECOM_LOG_GPS1_POS:
    {
      sensor_msgs::NavSatFix nav_msg;
      nav_msg.header.stamp = ros::Time::now();
      nav_msg.header.frame_id = "map";

      // GPS status
      nav_msg.status.status = 0;
      nav_msg.status.service = 0;
      if ((pLogData->gpsPosData.status & (SBG_ECOM_GPS_POS_STATUS_MASK << SBG_ECOM_GPS_POS_STATUS_SHIFT)) != SBG_ECOM_POS_SOL_COMPUTED)
        nav_msg.status.status = -1;
      if ((pLogData->gpsPosData.status & (SBG_ECOM_GPS_POS_TYPE_MASK << SBG_ECOM_GPS_POS_TYPE_SHIFT)) == SBG_ECOM_POS_NO_SOLUTION)
        nav_msg.status.status = -1;
      if (pLogData->gpsPosData.status & (0b111 << 12)) // using GPS
        nav_msg.status.service |= 1;
      if (pLogData->gpsPosData.status & (0b11 << 15)) // using GLONASS
        nav_msg.status.service |= 2;

      nav_msg.latitude = pLogData->gpsPosData.latitude;
      nav_msg.longitude = pLogData->gpsPosData.longitude;
      nav_msg.altitude = pLogData->gpsPosData.altitude + (double)pLogData->gpsPosData.undulation;

      // GPS position errors
      nav_msg.position_covariance[0] = pow(pLogData->gpsPosData.longitudeAccuracy,2);
      nav_msg.position_covariance[3] = pow(pLogData->gpsPosData.latitudeAccuracy,2);
      nav_msg.position_covariance[6] = pow(pLogData->gpsPosData.altitudeAccuracy,2);

      node->gps_pub.publish(nav_msg);
      break;
    }

    case SBG_ECOM_LOG_IMU_DATA:
    {
      sensor_msgs::Imu imu_msg;
      imu_msg.header.stamp = ros::Time::now();
      imu_msg.header.frame_id = "map";

      imu_msg.linear_acceleration.x = pLogData->imuData.accelerometers[0];
      imu_msg.linear_acceleration.y = pLogData->imuData.accelerometers[1];
      imu_msg.linear_acceleration.z = pLogData->imuData.accelerometers[2];

      imu_msg.angular_velocity.x = pLogData->imuData.gyroscopes[0];
      imu_msg.angular_velocity.y = pLogData->imuData.gyroscopes[1];
      imu_msg.angular_velocity.z = pLogData->imuData.gyroscopes[2];

      node->imu_pub.publish(imu_msg);
      break;
    }

    case SBG_ECOM_LOG_UTC_TIME:
    {
      sensor_msgs::TimeReference time_msg;
      time_msg.header.stamp = ros::Time::now();
      ptime gps_time(date(pLogData->utcData.year, pLogData->utcData.month, pLogData->utcData.day),
                     time_duration(pLogData->utcData.hour, pLogData->utcData.minute, pLogData->utcData.second)); // + nanoseconds(pLogData->utcData.nanoSecond));
      time_msg.time_ref = ros::Time::fromBoost(gps_time);
      if (pLogData->utcData.status & SBG_ECOM_CLOCK_UTC_SYNC)
        time_msg.source = "gps";
      else
        time_msg.source = "internal";
      node->time_pub.publish(time_msg);
      break;
    }

    default:
      break;
  }
  return SBG_NO_ERROR;
}

SbgDriver::SbgDriver(ros::NodeHandle nh)
  : nh(nh)
  , imu_pub(nh.advertise<sensor_msgs::Imu>("imu/data", 10))
  , gps_pub(nh.advertise<sensor_msgs::NavSatFix>("navsat/fix", 10))
  , time_pub(nh.advertise<sensor_msgs::TimeReference>("navsat/time_reference", 10))
  , pose_pub(nh.advertise<geometry_msgs::PoseStamped>("imu/pose", 10))
{
  diagnostic_updater::Updater updater;

  std::string uart_port;
  int uart_baud_rate;

  nh.param<std::string>("uart_port", uart_port, "/dev/ttyUSB0");
  nh.param<int>("uart_baud_rate", uart_baud_rate, 115200);

  errorCode = sbgInterfaceSerialCreate(&sbgInterface, uart_port.c_str(), uart_baud_rate);
  if (errorCode != SBG_NO_ERROR)
  {
    ROS_FATAL_STREAM("sbgInterfaceSerialCreate Error: " << errorCode);
  }

  errorCode = sbgEComInit(&comHandle, &sbgInterface); // Init the SBG
  if (errorCode != SBG_NO_ERROR)
  {
    ROS_FATAL_STREAM("sbgEComInit Error: " << errorCode);
  }

  ROS_INFO_STREAM("connecting to " << uart_port << ":" << uart_baud_rate);

  errorCode = sbgEComCmdGetInfo(&comHandle, &deviceInfo); // Get device info
  if (errorCode != SBG_NO_ERROR)
  {
    ROS_FATAL_STREAM("sbgEComCmdGetInfo Error: " << errorCode);
  }

  ROS_INFO_STREAM("connected to " << deviceInfo.productCode << " serial no. " << deviceInfo.serialNumber);

  updater.setHardwareID(boost::lexical_cast<std::string>(deviceInfo.serialNumber));

  // ****************************** SBG Config ******************************
  // ToDo: improve configuration capabilities

  errorCode = sbgEComCmdOutputSetConf(&comHandle, SBG_ECOM_OUTPUT_PORT_A, SBG_ECOM_CLASS_LOG_ECOM_0, SBG_ECOM_LOG_EKF_QUAT, SBG_ECOM_OUTPUT_MODE_DIV_8);
  if (errorCode != SBG_NO_ERROR){ROS_WARN("sbgEComCmdOutputSetConf SBG_ECOM_LOG_EKF_QUAT Error");}

  errorCode = sbgEComCmdOutputSetConf(&comHandle, SBG_ECOM_OUTPUT_PORT_A, SBG_ECOM_CLASS_LOG_ECOM_0, SBG_ECOM_LOG_EKF_NAV, SBG_ECOM_OUTPUT_MODE_DIV_8);
  if (errorCode != SBG_NO_ERROR){ROS_WARN("sbgEComCmdOutputSetConf SBG_ECOM_LOG_EKF_NAV Error");}

  errorCode = sbgEComCmdOutputSetConf(&comHandle, SBG_ECOM_OUTPUT_PORT_A, SBG_ECOM_CLASS_LOG_ECOM_0, SBG_ECOM_LOG_IMU_DATA, SBG_ECOM_OUTPUT_MODE_DIV_8);
  if (errorCode != SBG_NO_ERROR){ROS_WARN("sbgEComCmdOutputSetConf SBG_ECOM_LOG_IMU_DATA Error");}

  //errorCode = sbgEComCmdOutputSetConf(&comHandle, SBG_ECOM_OUTPUT_PORT_A, SBG_ECOM_CLASS_LOG_ECOM_0, SBG_ECOM_LOG_SHIP_MOTION, SBG_ECOM_OUTPUT_MODE_DIV_8);
  //if (errorCode != SBG_NO_ERROR){ROS_WARN("sbgEComCmdOutputSetConf SBG_ECOM_LOG_SHIP_MOTION Error");}

  errorCode = sbgEComCmdOutputSetConf(&comHandle, SBG_ECOM_OUTPUT_PORT_A, SBG_ECOM_CLASS_LOG_ECOM_0, SBG_ECOM_LOG_GPS1_POS, SBG_ECOM_OUTPUT_MODE_NEW_DATA);
  if (errorCode != SBG_NO_ERROR)
  {
    char errorMsg[256];
    sbgEComErrorToString(errorCode, errorMsg);
    ROS_ERROR_STREAM("sbgEComCmdOutputSetConf SBG_ECOM_LOG_GPS1_POS Error: " << errorMsg);
  }

  // SAVE AND REBOOT
  errorCode = sbgEComCmdSettingsAction(&comHandle, SBG_ECOM_SAVE_SETTINGS);
  if (errorCode != SBG_NO_ERROR){ROS_WARN("sbgEComCmdSettingsAction Error");}

  ROS_DEBUG("CONFIGURATION DONE");

  // ************************** SBG Callback for data ************************
  bool test = false;
  sbgEComSetReceiveLogCallback(&comHandle, onLogReceived, this);
}

void SbgDriver::run()
{
  ROS_INFO("START RECEIVING DATA");

  ros::Rate loop_rate(25);
  while (ros::ok())
  {
    int errorCode = sbgEComHandle(&comHandle);

    ros::spinOnce();
    loop_rate.sleep();
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "sbg_ellipse");
  ros::NodeHandle priv("~");
  SbgDriver sbg(priv);

  ROS_DEBUG_STREAM("starting sbg_ellipse node");

  sbg.run();
}
