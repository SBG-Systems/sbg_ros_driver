# SBG Ellipse Messages

Header header

# Time since sensor is powered up μs
uint32 time_stamp

# Velocity (North, East, Down) direction m/s
geometry_msgs/Vector3 velocity

# 1 σ Velocity in (North, East, Down) direction accuracy m/s
geometry_msgs/Vector3 velocity_accuracy

# [Latitude (°), Longitude (°), Altitude (above Mean Sea Level in meters)] 
geometry_msgs/Vector3 position

# Altitude difference between the geoid and the Ellipsoid (WGS-84 Altitude – MSL Altitude)
# (Height above Ellipsoid = altitude + undulation)
float32 undulation

# Latitude, Longitude, Vertical accuracy m (1 σ)
geometry_msgs/Vector3 position_accuracy

#  Global solution status
SbgEkfStatus status