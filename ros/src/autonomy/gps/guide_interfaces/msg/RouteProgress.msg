# Progress of the robot along the current route.
# Published by route_follower on guide/route_progress, once per guide/pose (10 Hz).
# All distances are in metres; positions are measured in the route frame (enu
# since Step 8: metres east / north of the configured origin).

# stamp = stamp of the pose this was computed from; frame_id = the route frame ("enu")
std_msgs/Header header

# Which route this refers to: the route_id given to guide/set_route,
# or 0 for a route loaded from the route_file parameter at startup.
# Lets listeners ignore messages left over from a previous route.
uint32 route_id

# Index of the route segment the robot is projected onto (0 = first leg).
uint32 segment_index

# Distance along the route from its start to the robot's projected point ("s").
float64 distance_along

# Distance along the route from the projected point to the end.
float64 distance_remaining

# Total length of the route.
float64 total_length

# Signed sideways distance from the route at the projected point.
# Positive = robot is left of the route (looking in the direction of travel),
# negative = right. Counter-clockwise positive, as in REP 103.
float64 cross_track_error

# True once the robot has reached the end of the route. Stays true until a new route is loaded.
bool arrived

# --- Road crossings (all false / -1 / 0.0 on a route without crossings) ---

# True while the robot is stopped before the next crossing, waiting for the
# user's go (guide/confirm_crossing). Only the user releases a crossing.
bool waiting_at_crossing

# True while the robot is between the near and far kerb of a crossing the
# user has confirmed.
bool on_crossing

# Index (0-based, in route order) of the first crossing not yet confirmed,
# or -1 if no crossings are left.
int32 next_crossing_index

# That crossing's near kerb (start_s) minus the robot's distance_along.
# Negative once the robot is past the near kerb; 0.0 if no crossings are left.
float64 distance_to_next_crossing

# --- Step 8 ---

# True while the robot has been more than off_route_distance (10 m) from the
# route for off_route_time (5 s), until it is back within back_on_route_distance
# (5 m). Never judged on or at a crossing, or while the position is too uncertain.
bool off_route
