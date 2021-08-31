# SBG Ellipse Messages
Header header

#  Time since sensor is powered up [us]
uint32 time_stamp

# GPS velocity fix and status bitmask
SbgGpsVelStatus status

# GPS Time of Week [ms]
uint32 gps_tow

# Velocity [m/s]
# In NED convention:
#   X: North
#   Y: East
#   Z: Down
# In ENU convention:
#   X: East
#   Y: North
#   Z: Up
geometry_msgs/Vector3 velocity

# Velocity accuracy (1 sigma) [m/s]
# In NED convention:
#   X: North
#   Y: East
#   Z: Vertical
# In ENU convention:
#   X: East
#   Y: North
#   Z: Vertical
geometry_msgs/Vector3 velocity_accuracy

# True direction of motion over ground (0 to 360 deg)
# NED convention: clockwise starting with North (0 then X axis is pointing North)
# ENU convention: counterclockwise startin with East (0 then X axis is pointing East)
float32 course

# 1 sgima course accuracy
float32 course_acc
