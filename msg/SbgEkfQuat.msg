# SBG Ellipse Messages

Header header

#  Time since sensor is powered up us 
uint32 time_stamp

#  Quaternion parameter (ROS order X, Y, Z, W)
geometry_msgs/Quaternion quaternion

#  [Roll, Pitch, Yaw (heading)] angle accuracy (1 sigma) rad 
geometry_msgs/Vector3 accuracy

#  Global solution status
SbgEkfStatus status
