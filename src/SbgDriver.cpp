#include "sbg_driver/SbgDriver.h"

#include "boost/date_time/posix_time/posix_time_types.hpp"
#include <diagnostic_updater/diagnostic_updater.h>

using namespace sbg_driver;

/**
 * Class constructor
 * Will open the connection and register the callback
 */
SbgDriver::SbgDriver(ros::NodeHandle nh, SbgEComReceiveLogFunc callbackFunction)
        : nh(nh),
          imu_raw_pub(nh.advertise<sensor_msgs::Imu>("imu/data", 10)),
          gps_pos_pub(nh.advertise<sensor_msgs::NavSatFix>("navsat/fix", 10)),
          gps_vel_pub(nh.advertise<geometry_msgs::TwistWithCovarianceStamped>("navsat/vel", 10)),
          gps_head_pub(nh.advertise<geometry_msgs::Vector3Stamped>("navsat/heading", 10)),
          time_pub(nh.advertise<sensor_msgs::TimeReference>("navsat/time_reference", 10)),
          ekf_pose_pub(nh.advertise<nav_msgs::Odometry>("ekf/pose", 10)),
          ekf_quat_pub(nh.advertise<geometry_msgs::QuaternionStamped>("ekf/quat", 10))
{
    diagnostic_updater::Updater updater;

    std::string uart_port;
    int uart_baud_rate;

    nh.param<std::string>("uart_port", uart_port, "/dev/ttyUSB0");
    nh.param<int>("uart_baud_rate", uart_baud_rate, 115200);

    // Create the serial interface with the specify port and rate
    errorCode = sbgInterfaceSerialCreate(&sbgInterface, uart_port.c_str(), (uint32)uart_baud_rate);
    if (errorCode != SBG_NO_ERROR)
    {
        char errorMsg[256];
        sbgEComErrorToString(errorCode, errorMsg);
        ROS_FATAL_STREAM("sbgInterfaceSerialCreate Error: " << errorMsg);
    }

    // Init the SBG if it was a successful allocation
    errorCode = sbgEComInit(&comHandle, &sbgInterface);
    if (errorCode != SBG_NO_ERROR)
    {
        char errorMsg[256];
        sbgEComErrorToString(errorCode, errorMsg);
        ROS_FATAL_STREAM("sbgEComInit Error: " << errorMsg);
    }

    // Debug message
    ROS_INFO_STREAM("Connecting to " << uart_port << ":" << uart_baud_rate);

    // Get the connected device info
    errorCode = sbgEComCmdGetInfo(&comHandle, &deviceInfo);
    if (errorCode != SBG_NO_ERROR)
    {
        char errorMsg[256];
        sbgEComErrorToString(errorCode, errorMsg);
        ROS_FATAL_STREAM("sbgEComCmdGetInfo Error: " << errorMsg);
    }

    // Debug message, and set our serial number
    ROS_INFO_STREAM("connected to " << deviceInfo.productCode << " serial no. " << deviceInfo.serialNumber);
    updater.setHardwareID(boost::lexical_cast<std::string>(deviceInfo.serialNumber));


    // ****************************** SBG Config ******************************
    // ToDo: improve configuration capabilities

    // Enable getting EKF quaternion
    errorCode = sbgEComCmdOutputSetConf(&comHandle, SBG_ECOM_OUTPUT_PORT_A, SBG_ECOM_CLASS_LOG_ECOM_0, SBG_ECOM_LOG_EKF_QUAT, SBG_ECOM_OUTPUT_MODE_DIV_4);
    if (errorCode != SBG_NO_ERROR) handleCmdErr("SBG_ECOM_LOG_EKF_QUAT", errorCode);

    // Enable getting EKF position and velocity in the NED coordinates
    errorCode = sbgEComCmdOutputSetConf(&comHandle, SBG_ECOM_OUTPUT_PORT_A, SBG_ECOM_CLASS_LOG_ECOM_0, SBG_ECOM_LOG_EKF_NAV, SBG_ECOM_OUTPUT_MODE_DIV_4);
    if (errorCode != SBG_NO_ERROR) handleCmdErr("SBG_ECOM_LOG_EKF_NAV", errorCode);

    // Enable getting IMU acceleration and angular velocity values
    errorCode = sbgEComCmdOutputSetConf(&comHandle, SBG_ECOM_OUTPUT_PORT_A, SBG_ECOM_CLASS_LOG_ECOM_0, SBG_ECOM_LOG_IMU_DATA, SBG_ECOM_OUTPUT_MODE_DIV_4);
    if (errorCode != SBG_NO_ERROR) handleCmdErr("SBG_ECOM_LOG_IMU_DATA", errorCode);

    // Enable getting magnetometer data
    //errorCode = sbgEComCmdOutputSetConf(&comHandle, SBG_ECOM_OUTPUT_PORT_A, SBG_ECOM_CLASS_LOG_ECOM_0, SBG_ECOM_LOG_MAG, SBG_ECOM_OUTPUT_MODE_DIV_4);
    //if (errorCode != SBG_NO_ERROR) handleCmdErr("SBG_ECOM_LOG_MAG", errorCode);

    // Enable getting GPS position
    errorCode = sbgEComCmdOutputSetConf(&comHandle, SBG_ECOM_OUTPUT_PORT_A, SBG_ECOM_CLASS_LOG_ECOM_0, SBG_ECOM_LOG_GPS1_POS, SBG_ECOM_OUTPUT_MODE_NEW_DATA);
    if (errorCode != SBG_NO_ERROR) handleCmdErr("SBG_ECOM_LOG_GPS1_POS", errorCode);

    // Enable getting GPS velocity
    errorCode = sbgEComCmdOutputSetConf(&comHandle, SBG_ECOM_OUTPUT_PORT_A, SBG_ECOM_CLASS_LOG_ECOM_0, SBG_ECOM_LOG_GPS1_VEL, SBG_ECOM_OUTPUT_MODE_NEW_DATA);
    if (errorCode != SBG_NO_ERROR) handleCmdErr("SBG_ECOM_LOG_GPS1_VEL", errorCode);

    // Enable getting GPS heading for a dual antenna system
    errorCode = sbgEComCmdOutputSetConf(&comHandle, SBG_ECOM_OUTPUT_PORT_A, SBG_ECOM_CLASS_LOG_ECOM_0, SBG_ECOM_LOG_GPS1_HDT, SBG_ECOM_OUTPUT_MODE_NEW_DATA);
    if (errorCode != SBG_NO_ERROR) handleCmdErr("SBG_ECOM_LOG_GPS1_HDT", errorCode);



    // ****************************** SBG Saving ******************************
    // Save our settings and reboot
    errorCode = sbgEComCmdSettingsAction(&comHandle, SBG_ECOM_SAVE_SETTINGS);
    if (errorCode != SBG_NO_ERROR){
        char errorMsg[256];
        sbgEComErrorToString(errorCode, errorMsg);
        ROS_WARN_STREAM("sbgEComCmdSettingsAction Error" << errorMsg);
    }


    // Debug
    ROS_DEBUG("CONFIGURATION DONE");


    // ************************** SBG Callback for data ************************
    // Note that this is the global function that is passed into this class
    errorCode = sbgEComSetReceiveLogCallback(&comHandle, callbackFunction, this);
    if (errorCode != SBG_NO_ERROR){
        char errorMsg[256];
        sbgEComErrorToString(errorCode, errorMsg);
        ROS_ERROR_STREAM("sbgEComSetReceiveLogCallback Error" << errorMsg);
    }
}

/**
 * Deconstructor
 * Will shutdown the serial interface with the device
 */
SbgDriver::~SbgDriver() {
    sbgInterfaceSerialDestroy(&sbgInterface);
};


/**
 * This will run the main node
 * Will make sure that we are reciving data and will check for errors
 */
void SbgDriver::run()
{
    ROS_INFO("START RECEIVING DATA");

    ros::Rate loop_rate(25);
    while (ros::ok())
    {
        SbgErrorCode errorCode = sbgEComHandle(&comHandle);
        if (errorCode != SBG_NO_ERROR){
            char errorMsg[256];
            sbgEComErrorToString(errorCode, errorMsg);
            ROS_ERROR_STREAM("sbgEComHandle Error" << errorMsg);
        }

        ros::spinOnce();
        loop_rate.sleep();
    }
}

/**
 * Helper method that handles printing out errors
 * Is just used when enabling things on the hardware
 */
void SbgDriver::handleCmdErr(std::string method, SbgErrorCode errorCode) {
    char errorMsg[256];
    sbgEComErrorToString(errorCode, errorMsg);
    ROS_ERROR_STREAM("Error in method = " << method << "\n" << errorMsg);
}
