# SBG Ellipse Messages
Header header

#  Time since sensor is powered up us 
uint32 time_stamp

# GPS velocity fix and status bitmask - 
SbgGpsVelStatus status

# GPS Time of Week ms 
uint32 gps_tow

# Velocity in [North, East, Down] direction m/s 
geometry_msgs/Vector3 vel

# 1 sgima Accuracy in [North, East, Down] direction m/s 
geometry_msgs/Vector3 vel_acc

# True direction of motion over ground (0 to 360 deg)
float32 course

# 1 sgima course accuracy (0 to 360 deg).
float32 course_acc
