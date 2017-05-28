#pragma once

#include "ros/ros.h"

#include "nav_msgs/Odometry.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/NavSatFix.h"
#include "sensor_msgs/TimeReference.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/QuaternionStamped.h"
#include "geometry_msgs/Vector3Stamped.h"
#include "geometry_msgs/TwistWithCovarianceStamped.h"

#include <sbgEComLib.h>
#include <sbgEComIds.h>

namespace sbg_driver
{
    class SbgDriver
    {
    public:

        // Constructors
        SbgDriver(ros::NodeHandle nh);
        ~SbgDriver();

        // Our publishers
        ros::Publisher imu_raw_pub;
        ros::Publisher imu_head_pub; //todo magnitometer data
        ros::Publisher gps_pos_pub;
        ros::Publisher gps_vel_pub;
        ros::Publisher gps_head_pub;
        ros::Publisher ekf_pose_pub;
        ros::Publisher ekf_quat_pub;
        ros::Publisher time_pub;

        // Methods
        void run();
        void handleCmdErr(std::string method, SbgErrorCode error);

        // Global function for log callbacks
        // Does this really need to be a global function?
        static SbgErrorCode onLogReceived(SbgEComHandle *pHandle, SbgEComClass msgClass, SbgEComMsgId msg, const SbgBinaryLogData *pLogData, void *pUserArg);

    private:
        ros::NodeHandle nh;

        SbgEComHandle       comHandle;
        SbgInterface        sbgInterface;
        SbgEComDeviceInfo   deviceInfo;
        SbgErrorCode        errorCode;
    };

}
