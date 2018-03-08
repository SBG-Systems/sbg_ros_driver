#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/NavSatFix.h"
#include "geometry_msgs/PoseStamped.h"
#include <sbgEComLib.h>
#include <sbgEComIds.h>

#include "ellipse_msg.h"

sbg_driver::SbgStatus sbgStatus_msg;
sbg_driver::SbgUtcTime sbgUtcTime_msg;
sbg_driver::SbgImuData sbgImuData_msg;
sbg_driver::SbgEkfEuler sbgEkfEuler_msg;
sbg_driver::SbgEkfQuat sbgEkfQuat_msg;
sbg_driver::SbgEkfNav sbgEkfNav_msg;
sbg_driver::SbgShipMotion sbgShipMotion_msg;
sbg_driver::SbgMag sbgMag_msg;
sbg_driver::SbgMagCalib sbgMagCalib_msg;
sbg_driver::SbgGpsVel sbgGpsVel_msg;
sbg_driver::SbgGpsPos sbgGpsPos_msg;
sbg_driver::SbgGpsHdt sbgGpsHdt_msg;
sbg_driver::SbgGpsRaw sbgGpsRaw_msg;
sbg_driver::SbgOdoVel sbgOdoVel_msg;
sbg_driver::SbgEvent sbgEvent_msg;
sbg_driver::SbgPressure sbgPressure_msg;

// sensor_msgs::Imu imu_msg;
// sensor_msgs::NavSatFix nav_msg;
// geometry_msgs::PoseStamped pose_msg;
// bool new_imu_msg;
// bool new_nav_msg;
// bool new_twist_msg;

bool new_sbgStatus;
bool new_sbgUtcTime;
bool new_sbgImuData;
bool new_sbgEkfEuler;
bool new_sbgEkfQuat;
bool new_sbgEkfNav;
bool new_sbgShipMotion;
bool new_sbgMag;
bool new_sbgMagCalib;
bool new_sbgGpsVel;
bool new_sbgGpsPos;
bool new_sbgGpsHdt;
bool new_sbgGpsRaw;
bool new_sbgOdoVel;
bool new_sbgEvent;
bool new_sbgPressure;

/*!
 *  Callback definition called each time a new log is received.
 *  \param[in]  pHandle                 Valid handle on the sbgECom instance that has called this callback.
 *  \param[in]  msgClass                Class of the message we have received
 *  \param[in]  msg                   Message ID of the log received.
 *  \param[in]  pLogData                Contains the received log data as an union.
 *  \param[in]  pUserArg                Optional user supplied argument.
 *  \return                       SBG_NO_ERROR if the received log has been used successfully.
 */
SbgErrorCode onLogReceived(SbgEComHandle *pHandle, SbgEComClass msgClass, SbgEComMsgId msg, const SbgBinaryLogData *pLogData, void *pUserArg)
{
  // float time_of_week;
  switch (msg){
    case SBG_ECOM_LOG_STATUS:
      read_ecom_log_status(sbgStatus_msg, pLogData);
      new_sbgStatus = true;
      break;

    case SBG_ECOM_LOG_UTC_TIME:
      read_ecom_log_utc_time(sbgUtcTime_msg, pLogData);
      new_sbgUtcTime = true;
      break;

    case SBG_ECOM_LOG_IMU_DATA:
      read_ecom_log_imu_data(sbgImuData_msg, pLogData);
      new_sbgImuData = true;
      break;

    case SBG_ECOM_LOG_EKF_EULER:
      read_ecom_log_ekf_euler(sbgEkfEuler_msg, pLogData);
      new_sbgEkfEuler = true;
      break;

    case SBG_ECOM_LOG_EKF_QUAT:
      read_ecom_log_ekf_quat(sbgEkfQuat_msg, pLogData);
      new_sbgEkfQuat = true;
      break;

    case SBG_ECOM_LOG_EKF_NAV:
      read_ecom_log_ekf_nav(sbgEkfNav_msg, pLogData);
      new_sbgEkfNav = true;
      break;

    case SBG_ECOM_LOG_SHIP_MOTION:
      read_ecom_log_ship_motion(sbgShipMotion_msg, pLogData);
      new_sbgShipMotion = true;
      break;

    case SBG_ECOM_LOG_MAG:
      read_ecom_log_mag(sbgMag_msg, pLogData);
      new_sbgMag = true;
      break;

    case SBG_ECOM_LOG_MAG_CALIB:
      read_ecom_log_mag_calib(sbgMagCalib_msg, pLogData);
      new_sbgMagCalib = true;
      break;

    case SBG_ECOM_LOG_GPS1_VEL:
      read_ecom_log_gps_vel(sbgGpsVel_msg, pLogData);
      new_sbgGpsVel = true;
      break;

    case SBG_ECOM_LOG_GPS1_POS:
      read_ecom_log_gps_pos(sbgGpsPos_msg, pLogData);
      new_sbgGpsPos = true;
      break;

    case SBG_ECOM_LOG_GPS1_HDT:
      read_ecom_log_gps_hdt(sbgGpsHdt_msg, pLogData);
      new_sbgGpsHdt = true;
      break;

    case SBG_ECOM_LOG_GPS1_RAW:
      read_ecom_log_gps_raw(sbgGpsRaw_msg, pLogData);
      new_sbgGpsRaw = true;
      break;

    case SBG_ECOM_LOG_ODO_VEL:
      read_ecom_log_odo_vel(sbgOdoVel_msg, pLogData);
      new_sbgOdoVel = true;
      break;

    case SBG_ECOM_LOG_EVENT_A:
      read_ecom_log_event(sbgEvent_msg, pLogData);
      new_sbgEvent = true;
      break;

    case SBG_ECOM_LOG_PRESSURE:
      read_ecom_log_pressure(sbgPressure_msg, pLogData);
      new_sbgPressure = true;
      break;

    default:
      break;
  }
  return SBG_NO_ERROR;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "sbg_ellipse");

  ros::NodeHandle n;
  ros::Publisher imu_pub = n.advertise<sensor_msgs::Imu>("imu", 10);
  ros::Publisher gps_pub = n.advertise<sensor_msgs::NavSatFix>("fix", 10);
  ros::Publisher pose_pub = n.advertise<geometry_msgs::PoseStamped>("imu_pose", 10);

  std::string uart_port;
  int uart_baud_rate;

  n.param<std::string>("uart_port", uart_port, "/dev/ttyUSB0");
  n.param<int>("uart_baud_rate", uart_baud_rate, 115200);

    // ********************* Initialize the SBG  *********************
  SbgEComHandle       comHandle;
  SbgInterface        sbgInterface;
  SbgEComDeviceInfo   deviceInfo;
  SbgErrorCode        errorCode;

  errorCode = sbgInterfaceSerialCreate(&sbgInterface, uart_port.c_str(), uart_baud_rate);
  if (errorCode != SBG_NO_ERROR){ROS_WARN("sbgInterfaceSerialCreate Error");}

  errorCode = sbgEComInit(&comHandle, &sbgInterface); // Init the SBG
  if (errorCode != SBG_NO_ERROR){ROS_WARN("sbgEComInit Error");}

  errorCode = sbgEComCmdGetInfo(&comHandle, &deviceInfo); // Get device info
  if (errorCode != SBG_NO_ERROR){ROS_WARN("sbgEComCmdGetInfo Error");}

  ROS_INFO("CONNEXTION SET-UP");

  // ****************************** SBG Config ******************************
  // ToDo: improve configuration capabilities

  errorCode = sbgEComCmdOutputSetConf(&comHandle, SBG_ECOM_OUTPUT_PORT_A, SBG_ECOM_CLASS_LOG_ECOM_0, SBG_ECOM_LOG_EKF_QUAT, SBG_ECOM_OUTPUT_MODE_DIV_8);
  if (errorCode != SBG_NO_ERROR){ROS_WARN("sbgEComCmdOutputSetConf SBG_ECOM_LOG_EKF_QUAT Error");}

  errorCode = sbgEComCmdOutputSetConf(&comHandle, SBG_ECOM_OUTPUT_PORT_A, SBG_ECOM_CLASS_LOG_ECOM_0, SBG_ECOM_LOG_EKF_NAV, SBG_ECOM_OUTPUT_MODE_DIV_8);
  if (errorCode != SBG_NO_ERROR){ROS_WARN("sbgEComCmdOutputSetConf SBG_ECOM_LOG_EKF_NAV Error");}

  errorCode = sbgEComCmdOutputSetConf(&comHandle, SBG_ECOM_OUTPUT_PORT_A, SBG_ECOM_CLASS_LOG_ECOM_0, SBG_ECOM_LOG_IMU_DATA, SBG_ECOM_OUTPUT_MODE_DIV_8);
  if (errorCode != SBG_NO_ERROR){ROS_WARN("sbgEComCmdOutputSetConf SBG_ECOM_LOG_IMU_DATA Error");}

  errorCode = sbgEComCmdOutputSetConf(&comHandle, SBG_ECOM_OUTPUT_PORT_A, SBG_ECOM_CLASS_LOG_ECOM_0, SBG_ECOM_LOG_SHIP_MOTION, SBG_ECOM_OUTPUT_MODE_DIV_8);
  if (errorCode != SBG_NO_ERROR){ROS_WARN("sbgEComCmdOutputSetConf SBG_ECOM_LOG_SHIP_MOTION Error");}

  // SAVE AND REBOOT
  errorCode = sbgEComCmdSettingsAction(&comHandle, SBG_ECOM_SAVE_SETTINGS);
  if (errorCode != SBG_NO_ERROR){ROS_WARN("sbgEComCmdSettingsAction Error");}

  ROS_INFO("CONFIGURATION DONE");

  // ************************** SBG Callback for data ************************
  bool test = false;
  sbgEComSetReceiveLogCallback(&comHandle, onLogReceived, NULL);

  ROS_INFO("START RECEIVING DATA");

  // imu_msg.header.frame_id = "map";
  // nav_msg.header.frame_id = "map";
  // pose_msg.header.frame_id = "map";

  ros::Rate loop_rate(25);
  while (ros::ok())
  {
    int errorCode = sbgEComHandle(&comHandle);

    // if(new_nav_msg){
    //   nav_msg.header.stamp = ros::Time::now();
    //   gps_pub.publish(nav_msg);  
    //   new_nav_msg = false;
    // }

    // if(new_imu_msg){
    //   imu_msg.header.stamp = ros::Time::now();
    //   imu_pub.publish(imu_msg);
    //   pose_msg.header.stamp = ros::Time::now();
    //   pose_pub.publish(pose_msg);

    //   new_imu_msg = false;
    // }

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}