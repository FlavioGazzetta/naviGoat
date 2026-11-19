# One road crossing along a planned route.
uint8 SIGNALLED=0
uint8 UNSIGNALLED=1
uint8 UNKNOWN=2

float64 start_s          # distance along the route (m) of the near kerb
float64 end_s            # distance along the route (m) of the far kerb
uint8 crossing_type      # SIGNALLED, UNSIGNALLED or UNKNOWN
string crossed_road      # 'major', 'minor', 'none' (no public road, e.g. cycle track) or 'unknown'
bool tactile_paving
bool has_island          # part of a two-stage crossing with a pedestrian island