#include "sbg_driver/SbgDriver.h"

#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/NavSatFix.h"
#include "sensor_msgs/TimeReference.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/QuaternionStamped.h"
#include "geometry_msgs/Vector3Stamped.h"
#include "geometry_msgs/TwistWithCovarianceStamped.h"
#include <diagnostic_updater/diagnostic_updater.h>


// Global function for log callbacks
// Does this really need to be a global function?
// Not sure, but it doesn't work when it is in a class
SbgErrorCode onLogReceived(SbgEComHandle *pHandle, SbgEComClass msgClass, SbgEComMsgId msg, const SbgBinaryLogData *pLogData, void *pUserArg);


/**
 * Main function of the driver
 * Will create the driver and run it
 */
int main(int argc, char **argv)
{
    // Init our node
    ros::init(argc, argv, "sbg_ellipse");
    ros::NodeHandle priv("~");
    sbg_driver::SbgDriver sbg(priv, onLogReceived);

    // Run the program
    ROS_DEBUG_STREAM("starting sbg_ellipse node");
    sbg.run();
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
SbgErrorCode onLogReceived(SbgEComHandle *pHandle, SbgEComClass msgClass, SbgEComMsgId msg, const SbgBinaryLogData *pLogData, void *pUserArg)
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
