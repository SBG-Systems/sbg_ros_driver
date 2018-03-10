#include "ros/ros.h"
#include "ellipse.h"


int main(int argc, char **argv)
{
  ros::init(argc, argv, "sbg_ellipse");
  ros::NodeHandle n;

  ROS_INFO("SBG DRIVER - Init node & load params");
  Ellipse ellipse(&n);

  ellipse.init_publishers();

  ROS_INFO("SBG DRIVER - Ellipse connect");
  ellipse.connect();

  ROS_INFO("SBG DRIVER - Ellipse configure");
  ellipse.configure();

  ROS_INFO("SBG DRIVER - Init callback");
  ellipse.init_callback();

  ROS_INFO("SBG DRIVER - START RECEIVING DATA");
  ros::Rate loop_rate(ellipse.m_rate_frequency);
  while (ros::ok())
  {
    ellipse.publish();

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
