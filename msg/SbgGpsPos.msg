# SBG Ellipse Messages
Header header

#  Time since sensor is powered up us 
uint32 time_stamp

# GPS position fix and status bitmask
SbgGpsPosStatus status

# GPS Time of Week ms
uint32 gps_tow

# [Latitude (positive North deg), Longitude (positive East deg), Altitude (Above Mean Sea Level m)]
geometry_msgs/Vector3 position

# Altitude difference between the geoid and the Ellipsoid (WGS-84 Altitude - MSL Altitude)
# (Height above Ellipsoid = altitude + undulation)
float32 undulation

# Latitude, Longitude, Vertical accuracy m (1 sigma)
geometry_msgs/Vector3 position_accuracy

# Number of space vehicles used in GNSS solution
uint8 num_sv_used

# ID of the DGPS/RTK base station in use
uint16 base_station_id

# Differential data age 0.01 s
uint16 diff_age
