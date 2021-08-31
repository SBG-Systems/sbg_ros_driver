# SBG Ellipse Messages

Header header

# Time since sensor is powered up micro [us]
uint32 time_stamp

# Angle [Roll, Pitch, Yaw (heading)] [rad]
#
# NED convention:
#   Roll: Clockwise around X axis
#   Pitch: Clockwise around Y axis
#   Yaw: Clockwise starting with North
#
# ENU convention:
#   Roll: Clockwise around X axis
#   Pitch: Counter-clockwise around Y axis
#   Yaw: Counter-clockwise startin with East
geometry_msgs/Vector3 angle

# Angle accuracy (Roll, Pitch, Yaw (heading)) (1 sigma) [rad]
geometry_msgs/Vector3 accuracy

#  Global solution status
SbgEkfStatus status
