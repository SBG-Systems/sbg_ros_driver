#include <ellipse.h>

using sbg::Ellipse;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "sbg_ellipse");
  ros::NodeHandle n;

  try
  {
    ROS_INFO("SBG DRIVER - Init node & load params");
    Ellipse ellipse(&n);

    ROS_INFO("SBG DRIVER - Ellipse connect");
    ellipse.connect();

    ROS_INFO("SBG DRIVER - Ellipse configure for receiving data");
    ellipse.initEllipseForReceivingData();

    ROS_INFO("SBG DRIVER - START RECEIVING DATA");
    ros::Rate loop_rate(ellipse.getDeviceRateFrequency());

    while (ros::ok())
    {
      ellipse.periodicHandle();
      loop_rate.sleep();
    }

    return 0;
  }
  catch (ros::Exception const& refE)
  {
    ROS_ERROR("SBG_DRIVER - [Error] %s", refE.what());
  }

  return 0;
}