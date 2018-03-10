#include "ros/ros.h"
#include "ellipse.h"


int main(int argc, char **argv)
{
  ros::init(argc, argv, "sbg_ellipse");
  ros::NodeHandle n;

  ROS_INFO("init node");
  Ellipse ellipse(&n);

  ROS_INFO("init publishers");
  ellipse.init_publishers();

  ROS_INFO("ellipse connect");
  ellipse.connect();

  ROS_INFO("init callback");
  ellipse.init_callback();

  // std::string uart_port;
  // int uart_baud_rate;

  // n.param<std::string>("uart_port", uart_port, "/dev/ttyUSB0");
  // n.param<int>("uart_baud_rate", uart_baud_rate, 115200);

  //   // ********************* Initialize the SBG  *********************
  // SbgEComHandle       comHandle;
  // SbgInterface        sbgInterface;
  // SbgEComDeviceInfo   deviceInfo;
  // SbgErrorCode        errorCode;

  // errorCode = sbgInterfaceSerialCreate(&sbgInterface, uart_port.c_str(), uart_baud_rate);
  // if (errorCode != SBG_NO_ERROR){ROS_WARN("sbgInterfaceSerialCreate Error");}

  // errorCode = sbgEComInit(&comHandle, &sbgInterface); // Init the SBG
  // if (errorCode != SBG_NO_ERROR){ROS_WARN("sbgEComInit Error");}

  // errorCode = sbgEComCmdGetInfo(&comHandle, &deviceInfo); // Get device info
  // if (errorCode != SBG_NO_ERROR){ROS_WARN("sbgEComCmdGetInfo Error");}

  // ROS_INFO("CONNEXTION SET-UP");

  // // ****************************** SBG Config ******************************
  // // ToDo: improve configuration capabilities

  // errorCode = sbgEComCmdOutputSetConf(&comHandle, SBG_ECOM_OUTPUT_PORT_A, SBG_ECOM_CLASS_LOG_ECOM_0, SBG_ECOM_LOG_EKF_QUAT, SBG_ECOM_OUTPUT_MODE_DIV_8);
  // if (errorCode != SBG_NO_ERROR){ROS_WARN("sbgEComCmdOutputSetConf SBG_ECOM_LOG_EKF_QUAT Error");}

  // errorCode = sbgEComCmdOutputSetConf(&comHandle, SBG_ECOM_OUTPUT_PORT_A, SBG_ECOM_CLASS_LOG_ECOM_0, SBG_ECOM_LOG_EKF_NAV, SBG_ECOM_OUTPUT_MODE_DIV_8);
  // if (errorCode != SBG_NO_ERROR){ROS_WARN("sbgEComCmdOutputSetConf SBG_ECOM_LOG_EKF_NAV Error");}

  // errorCode = sbgEComCmdOutputSetConf(&comHandle, SBG_ECOM_OUTPUT_PORT_A, SBG_ECOM_CLASS_LOG_ECOM_0, SBG_ECOM_LOG_IMU_DATA, SBG_ECOM_OUTPUT_MODE_DIV_8);
  // if (errorCode != SBG_NO_ERROR){ROS_WARN("sbgEComCmdOutputSetConf SBG_ECOM_LOG_IMU_DATA Error");}

  // errorCode = sbgEComCmdOutputSetConf(&comHandle, SBG_ECOM_OUTPUT_PORT_A, SBG_ECOM_CLASS_LOG_ECOM_0, SBG_ECOM_LOG_SHIP_MOTION, SBG_ECOM_OUTPUT_MODE_DIV_8);
  // if (errorCode != SBG_NO_ERROR){ROS_WARN("sbgEComCmdOutputSetConf SBG_ECOM_LOG_SHIP_MOTION Error");}

  // // SAVE AND REBOOT
  // errorCode = sbgEComCmdSettingsAction(&comHandle, SBG_ECOM_SAVE_SETTINGS);
  // if (errorCode != SBG_NO_ERROR){ROS_WARN("sbgEComCmdSettingsAction Error");}

  // ROS_INFO("CONFIGURATION DONE");

  // // ************************** SBG Callback for data ************************
  
  // // void (*onLogReceived)(SbgEComHandle*, SbgEComClass, SbgEComMsgId, const SbgBinaryLogData*, void*);
  // // onLogReceived = &ellipse::onLogReceived;

  // // sbgEComSetReceiveLogCallback(&comHandle, onLogReceived, NULL);

  ROS_INFO("START RECEIVING DATA");

  ros::Rate loop_rate(25);
  while (ros::ok())
  {
    ellipse.publish();

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}