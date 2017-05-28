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


/**
 * Main function of the driver
 * Will create the driver and run it
 */
int main(int argc, char **argv)
{
    // Init our node
    ros::init(argc, argv, "sbg_ellipse");
    ros::NodeHandle priv("~");
    sbg_driver::SbgDriver sbg(priv);

    // Run the program
    ROS_DEBUG_STREAM("starting sbg_ellipse node");
    sbg.run();
}

