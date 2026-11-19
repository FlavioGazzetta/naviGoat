# Status of the NavigateToPose goal currently handled by the goal sender.
# Published on guide/goal_status with latched QoS (reliable + transient_local, depth 1)
# every time the status changes, so late-starting nodes still get the latest state.

# Status values. In a .msg file, NAME=value lines are constants: they are not sent
# in each message, they become class attributes (e.g. NavGoalStatus.SUCCEEDED in Python).
uint8 PENDING=0     # goal received; waiting for the navigate_to_pose server or for accept/reject
uint8 ACTIVE=1      # accepted by Nav2; robot is driving to it
uint8 SUCCEEDED=2   # Nav2 reached the goal
uint8 ABORTED=3     # Nav2 failed (not a preemption by our own newer goal; those are filtered out)
uint8 CANCELED=4    # someone cancelled the goal
uint8 REJECTED=5    # Nav2 refused the goal

# stamp = time the status changed; frame_id = map.
std_msgs/Header header

# The goal exactly as received on guide/next_goal, original stamp included.
# The route follower matches it against its own current goal (safety net).
geometry_msgs/PoseStamped goal

# One of the constants above.
uint8 status

# Nav2's result error code (0 = no error) and a human-readable detail.
uint16 error_code
string message