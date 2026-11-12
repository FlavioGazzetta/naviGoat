"""What Pete can be asked for. The middleman only accepts these; anything else gets a spoken "unknown" reply.

Edit the lists here: add a place that is on the map, or an object the detector should look for.
"""

# Places stored on the map (NAVIGATION). Replace with the real names from the map.
LOCATIONS = ['bathroom', 'kitchen', 'exit', 'entrance', 'lobby']
LOCATION_ALIASES = {'restroom': 'bathroom', 'washroom': 'bathroom', 'toilets': 'bathroom'}

# Objects the detector looks for. Keep in sync with VOCABULARY in ros/src/autonomy/perception/classification.py.
OBJECTS = ['person', 'chair', 'couch', 'bed', 'table', 'door', 'tv', 'remote', 'laptop', 'cell phone', 'bottle', 'cup', 'book', 'backpack', 'keys', 'wallet', 'glasses', 'shoe', 'refrigerator', 'sink', 'toilet', 'stairs', 'dog', 'cat']
OBJECT_ALIASES = {'television': 'tv', 'remote control': 'remote', 'tv remote': 'remote', 'sofa': 'couch', 'phone': 'cell phone', 'fridge': 'refrigerator', 'desk': 'table', 'key': 'keys', 'steps': 'stairs'}

# "Take me to the toilet" means the room, not the fixture, when that room is a stored location.
GUIDE_TO_LOCATION = {'toilet': 'bathroom'}
# If Pete cannot find the object while guiding, try this stored location instead.
OBJECT_FALLBACK_LOCATIONS = {'toilet': 'bathroom', 'door': 'exit', 'refrigerator': 'kitchen', 'sink': 'kitchen'}
