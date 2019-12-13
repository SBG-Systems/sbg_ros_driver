#include <sbg_device.h>

using sbg::SbgDevice;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "sbg_device");
  ros::NodeHandle node_handle;

  try
  {
    ROS_INFO("SBG DRIVER - Init node, load params and connect to the device.");
    SbgDevice sbg_device(node_handle);

    ROS_INFO("SBG DRIVER - Initialize device for receiving data");
    sbg_device.initDeviceForReceivingData();

    ROS_INFO("SBG DRIVER - START RECEIVING DATA");
    ros::Rate loop_rate(sbg_device.getUpdateFrequency() * 2);

    while (ros::ok())
    {
      sbg_device.periodicHandle();
      loop_rate.sleep();
    }

    return 0;
  }
  catch (ros::Exception const& refE)
  {
    ROS_ERROR("SBG_DRIVER - %s", refE.what());
  }

  return 0;
}