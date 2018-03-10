#include "ros/ros.h"
#include "ellipse.h"


int main(int argc, char **argv)
{
  ros::init(argc, argv, "sbg_ellipse");
  ros::NodeHandle n;

  ROS_INFO("Init node");
  Ellipse ellipse(&n);

  ROS_INFO("Init publishers");
  ellipse.init_publishers();

  ROS_INFO("Ellipse connect");
  ellipse.connect();

  ROS_INFO("Ellipse configure");
  ellipse.configure();

  ROS_INFO("Init callback");
  ellipse.init_callback();

  // std::string uart_port;
  // int uart_baud_rate;

  // n.param<std::string>("uart_port", uart_port, "/dev/ttyUSB0");
  // n.param<int>("uart_baud_rate", uart_baud_rate, 115200);

  ROS_INFO("START RECEIVING DATA");

  ros::Rate loop_rate(ellipse.m_rate_frequency);
  while (ros::ok())
  {
    ellipse.publish();

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
