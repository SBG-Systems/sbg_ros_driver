#include "sbg_driver/SbgDriver.h"

#include "boost/date_time/posix_time/posix_time_types.hpp"
#include <diagnostic_updater/diagnostic_updater.h>

using namespace sbg_driver;

/**
 * Class constructor
 * Will open the connection and register the callback
 */
SbgDriver::SbgDriver(ros::NodeHandle nh)
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
    errorCode = sbgEComSetReceiveLogCallback(&comHandle, SbgDriver::onLogReceived, this);
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



/*!
 *  Callback definition called each time a new log is received.
 *  \param[in]  pHandle                 Valid handle on the sbgECom instance that has called this callback.
 *  \param[in]  msgClass                Class of the message we have received
 *  \param[in]  msg                     Message ID of the log received.
 *  \param[in]  pLogData                Contains the received log data as an union.
 *  \param[in]  pUserArg                Optional user supplied argument.
 *  \return                             SBG_NO_ERROR if the received log has been used successfully.
 */
SbgErrorCode SbgDriver::onLogReceived(SbgEComHandle *pHandle, SbgEComClass msgClass, SbgEComMsgId msg, const SbgBinaryLogData *pLogData, void *pUserArg)
{
    // Cast the user to our class type
    sbg_driver::SbgDriver* node = static_cast<sbg_driver::SbgDriver*>(pUserArg);
    ROS_DEBUG_STREAM("sbgECom received log data id: " <<  (int)msg);

    // Master switch statement that handles each message type
    switch (msg){
        case SBG_ECOM_LOG_EKF_QUAT:
        {
            geometry_msgs::QuaternionStamped msg;
            msg.header.stamp = ros::Time::now();
            msg.header.frame_id = "map";
            // Orientation quaternion stored in W, X, Y, Z form.
            msg.quaternion.x = pLogData->ekfQuatData.quaternion[1];
            msg.quaternion.y = pLogData->ekfQuatData.quaternion[2];
            msg.quaternion.z = pLogData->ekfQuatData.quaternion[3];
            msg.quaternion.w = pLogData->ekfQuatData.quaternion[0];
            node->ekf_quat_pub.publish(msg);
            break;
        }

        case SBG_ECOM_LOG_EKF_NAV:
        {
            nav_msgs::Odometry msg;
            msg.header.stamp = ros::Time::now();
            msg.header.frame_id = "map";
            // Latitude, Longitude in degrees positive North and East. Altitude above Mean Sea Level in meters
            msg.pose.pose.position.x = pLogData->ekfNavData.position[0];
            msg.pose.pose.position.y = pLogData->ekfNavData.position[1];
            msg.pose.pose.position.z = pLogData->ekfNavData.position[2];
            msg.pose.covariance[0] = pow(pLogData->ekfNavData.positionStdDev[0],2);
            msg.pose.covariance[7] = pow(pLogData->ekfNavData.positionStdDev[1],2);
            msg.pose.covariance[14] = pow(pLogData->ekfNavData.positionStdDev[2],2);
            // North, East, Down velocity in m.s^-1.
            msg.twist.twist.linear.x = pLogData->ekfNavData.velocity[0];
            msg.twist.twist.linear.y = pLogData->ekfNavData.velocity[1];
            msg.twist.twist.linear.z = pLogData->ekfNavData.velocity[2];
            msg.twist.covariance[0] = pow(pLogData->ekfNavData.velocityStdDev[0],2);
            msg.twist.covariance[7] = pow(pLogData->ekfNavData.velocityStdDev[1],2);
            msg.twist.covariance[14] = pow(pLogData->ekfNavData.velocityStdDev[2],2);
            node->ekf_pose_pub.publish(msg);
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
            // TODO: Handle the different status better
            if ((pLogData->gpsPosData.status & (SBG_ECOM_GPS_POS_STATUS_MASK << SBG_ECOM_GPS_POS_STATUS_SHIFT)) != SBG_ECOM_POS_SOL_COMPUTED) {
                nav_msg.status.status = -1;
                break;
            }
            if ((pLogData->gpsPosData.status & (SBG_ECOM_GPS_POS_TYPE_MASK << SBG_ECOM_GPS_POS_TYPE_SHIFT)) == SBG_ECOM_POS_NO_SOLUTION)
                nav_msg.status.status = -1;
            if (pLogData->gpsPosData.status & (0b111 << 12)) // using GPS
                nav_msg.status.service |= 1;
            if (pLogData->gpsPosData.status & (0b11 << 15)) // using GLONASS
                nav_msg.status.service |= 2;
            // Set lat long and altitude
            nav_msg.latitude = pLogData->gpsPosData.latitude;
            nav_msg.longitude = pLogData->gpsPosData.longitude;
            nav_msg.altitude = pLogData->gpsPosData.altitude + (double)pLogData->gpsPosData.undulation;
            // GPS position errors (1 sigma longitude accuracy in meters)
            nav_msg.position_covariance[0] = pow(pLogData->gpsPosData.longitudeAccuracy,2);
            nav_msg.position_covariance[4] = pow(pLogData->gpsPosData.latitudeAccuracy,2);
            nav_msg.position_covariance[8] = pow(pLogData->gpsPosData.altitudeAccuracy,2);

            node->gps_pos_pub.publish(nav_msg);
            break;
        }

        case SBG_ECOM_LOG_GPS1_VEL:
        {
            geometry_msgs::TwistWithCovarianceStamped msg;
            msg.header.stamp = ros::Time::now();
            msg.header.frame_id = "map";
            // GPS North, East, Down velocity in m.s^-1.
            msg.twist.twist.linear.x = pLogData->gpsVelData.velocity[0];
            msg.twist.twist.linear.y = pLogData->gpsVelData.velocity[1];
            msg.twist.twist.linear.z = pLogData->gpsVelData.velocity[2];
            // GPS North, East, Down velocity 1 sigma accuracy in m.s^-1.
            msg.twist.covariance[0] = pow(pLogData->gpsVelData.velocityAcc[0],2);
            msg.twist.covariance[7] = pow(pLogData->gpsVelData.velocityAcc[1],2);
            msg.twist.covariance[14] = pow(pLogData->gpsVelData.velocityAcc[2],2);
            node->gps_vel_pub.publish(msg);
            break;
        }

        case SBG_ECOM_LOG_GPS1_HDT:
        {
            geometry_msgs::Vector3Stamped msg;
            msg.header.stamp = ros::Time::now();
            msg.header.frame_id = "map";

            // Check the status here
            if (pLogData->gpsHdtData.status & (SBG_ECOM_GPS_HDT_STATUS_MASK << SBG_ECOM_GPS_HDT_STATUS_SHIFT) == SBG_ECOM_HDT_INSUFFICIENT_OBS) {
                ROS_WARN_THROTTLE(120, "GPS HDT HEADING INSUFFICIENT OBSERVATIONS");
                break;
            }
            if (pLogData->gpsHdtData.status & (SBG_ECOM_GPS_HDT_STATUS_MASK << SBG_ECOM_GPS_HDT_STATUS_SHIFT) == SBG_ECOM_HDT_INTERNAL_ERROR) {
                ROS_ERROR_THROTTLE(120, "GPS HDT HEADING INTERNAL OBSERVATIONS");
                break;
            }
            if (pLogData->gpsHdtData.status & (SBG_ECOM_GPS_HDT_STATUS_MASK << SBG_ECOM_GPS_HDT_STATUS_SHIFT) == SBG_ECOM_HDT_HEIGHT_LIMIT) {
                ROS_WARN_THROTTLE(120, "GPS HDT HEADING HEIGHT LIMIT REACHED");
                break;
            }
            // GPS true heading in degrees.
            msg.vector.y = pLogData->gpsHdtData.pitch;
            msg.vector.z = pLogData->gpsHdtData.heading;
            node->gps_head_pub.publish(msg);
            break;
        }

        case SBG_ECOM_LOG_IMU_DATA:
        {
            sensor_msgs::Imu imu_msg;
            imu_msg.header.stamp = ros::Time::now();
            imu_msg.header.frame_id = "map";
            // X, Y, Z accelerometers in m.s^-2.
            imu_msg.linear_acceleration.x = pLogData->imuData.accelerometers[0];
            imu_msg.linear_acceleration.y = pLogData->imuData.accelerometers[1];
            imu_msg.linear_acceleration.z = pLogData->imuData.accelerometers[2];
            // X, Y, Z gyroscopes in rad.s^-1.
            imu_msg.angular_velocity.x = pLogData->imuData.gyroscopes[0];
            imu_msg.angular_velocity.y = pLogData->imuData.gyroscopes[1];
            imu_msg.angular_velocity.z = pLogData->imuData.gyroscopes[2];
            node->imu_raw_pub.publish(imu_msg);
            break;
        }

        case SBG_ECOM_LOG_UTC_TIME:
        {
            sensor_msgs::TimeReference time_msg;
            time_msg.header.stamp = ros::Time::now();
            // + nanoseconds(pLogData->utcData.nanoSecond));
            boost::posix_time::ptime gps_time(boost::gregorian::date(pLogData->utcData.year, pLogData->utcData.month, pLogData->utcData.day),
                                              boost::posix_time::time_duration(pLogData->utcData.hour, pLogData->utcData.minute, pLogData->utcData.second));
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

