# Status of localizer v1 (Step 8), published on guide/localization_status at 2 Hz.
# quality says whether the robot may guide normally (GOOD), should be careful
# (DEGRADED: GPS weak or absent, e.g. in a tunnel), or must stop (LOST).

uint8 GOOD=0
uint8 DEGRADED=1
uint8 LOST=2

std_msgs/Header header
uint8 quality
# Why, for logs (not meant to be spoken), e.g. "no GPS fix for 25 s".
string reason
# False until the first alignment of the map frame with GPS.
bool initialised
# Uncertainty (1 sigma) of the robot's position, largest axis (m); inf before init.
float64 position_sigma
# Uncertainty (1 sigma) of the robot's heading (rad); inf before init.
float64 yaw_sigma
# Since the last GPS fix the filter accepted (s, m); inf / 0 before the first.
float64 seconds_since_fix
float64 distance_since_fix
# GPS fixes the filter accepted / rejected as outliers since start.
uint32 fixes_accepted
uint32 fixes_rejected
