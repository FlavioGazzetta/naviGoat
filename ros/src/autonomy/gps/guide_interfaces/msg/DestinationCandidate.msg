# One possible match for a destination the user asked for.
# Returned (best first) by guide/resolve_destination.

uint8 EXACT=0                 # query equals the name or an alias (after normalising)
uint8 FUZZY=1                 # close spelling or partial match

string name                   # e.g. "Central Library"
string matched_text           # the name or alias that matched, e.g. "library"
uint8 match_type
float64 score                 # 0..1, 1 = exact
string note                   # e.g. "main entrance, step-free"
float64 latitude              # WGS84 degrees of the point to walk to (the entrance);
float64 longitude             # for the planner only, never read out to the user
float64 straight_line_distance  # metres from the robot as the crow flies; -1 if no recent pose
string source                 # where it came from: "curated" (later "osm", "nominatim")