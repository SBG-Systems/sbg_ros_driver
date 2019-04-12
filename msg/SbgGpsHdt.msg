# SBG Ellipse Messages
Header header

# Time since sensor is powered up us 
uint32 time_stamp

# GPS True Heading status.
# 0 SOL_COMPUTED		A valid solution has been computed.
# 1 INSUFFICIENT_OBS	Not enough valid SV to compute a solution.
# 2 INTERNAL_ERROR		An internal error has occurred.
# 3 HEIGHT_LIMIT		The height limit has been exceeded.
uint16 status

# GPS Time of Week ms
uint32 tow

# True heading angle (0 to 360 deg).
float32 true_heading

# 1 sigma True heading estimated accuracy (0 to 360 deg).
float32 true_heading_acc

# Pitch angle from the master to the rover
float32 pitch

# 1 sigma pitch estimated accuracy
float32 pitch_acc
