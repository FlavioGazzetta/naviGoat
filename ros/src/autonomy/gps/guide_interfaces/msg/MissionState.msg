# State of the mission manager, for the user interface (speech, app, logs).
# Published latched on guide/mission_state on every change, and embedded in
# NavigateTo feedback.

std_msgs/Header header

uint8 IDLE=0
uint8 PLANNING=1
uint8 READY=2                 # route planned, waiting for the user's go to set off
uint8 NAVIGATING=3
uint8 WAITING_AT_CROSSING=4   # stopped before a crossing, waiting for the user's go
uint8 PAUSED=5                # stopped by the user or after a Nav2 failure, waiting for go
uint8 ARRIVED=6
uint8 FAILED=7
# Step 8 (new states are always appended; unknown states: speak `message`)
uint8 GPS_DEGRADED=8          # navigating, but the position is uncertain (e.g. a tunnel)
uint8 POSITION_LOST=9         # stopped: position too uncertain, waiting for GPS
uint8 REROUTING=10            # left the route: stopped, planning a new one (then READY)
uint8 state

uint32 route_id               # id of the route handed to the follower (0 = none)
string destination_name       # e.g. "Central Library"; empty when IDLE
string destination_note       # e.g. "main entrance, step-free"
float64 route_length          # metres along the planned route; 0.0 before planning
uint32 num_crossings          # crossings on the planned route
string summary                # read-back sentence shown in READY
string[] warnings             # planner warnings, plus "GPS signal weak" / "position lost"
string message                # latest event as a sentence, e.g. "Paused."
