# SBG Ellipse Messages
Header header

# Time since sensor is powered up us 
uint32 time_stamp

# IMU Status
SbgImuStatus imu_status

# Filtered Accelerometer - X, Y, Z axis m/s2 
geometry_msgs/Vector3 accel

# Filtered Gyroscope - X, Y, Z axis rad/s 
geometry_msgs/Vector3 gyro

# Internal Temperature degC 
float32 temp

# Sculling output - X, Y, Z axis m/s2 
geometry_msgs/Vector3 delta_vel

# Coning output - X, Y, Z axis rad/s 
geometry_msgs/Vector3 delta_angle
