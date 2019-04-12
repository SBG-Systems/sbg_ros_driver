# SBG Ellipse Messages

Header header

#  Time since sensor is powered up micro s
uint32 time_stamp

#  [Roll, Pitch, Yaw (heading)] angle rad 
geometry_msgs/Vector3 angle

#  [Roll, Pitch, Yaw (heading)] angle accuracy (1 sigma) rad
geometry_msgs/Vector3 accuracy

#  Global solution status
SbgEkfStatus status
