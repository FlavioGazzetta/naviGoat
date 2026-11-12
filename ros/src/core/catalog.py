"""What Pete can be asked for. The middleman only accepts these; anything else gets a spoken "unknown" reply.

Edit the lists here: add a place that is on the map, or an object the detector should look for.
"""

# Places stored on the map (NAVIGATION). Names only: each one still needs a place saved on the map before the robot can go there.
# Write them without a leading "the", because the middleman strips it from what was said.
LOCATIONS = [
    # any building
    'bathroom', 'kitchen', 'exit', 'entrance', 'lobby',
    # rooms in a house (bathroom, kitchen and office are above and below)
    'bedroom', 'living room', 'dining room', 'hallway', 'study', 'garage', 'garden', 'laundry room',
    # rooms found in a campus building
    'lecture theatre', 'seminar room', 'classroom', 'laboratory', 'computer room', 'study room', 'common room',
    'meeting room', 'office', 'reception', 'corridor', 'prayer room', 'canteen', 'cafe',
    # Imperial College, South Kensington (names from Imperial's campus maps and venue pages)
    'sherfield building', 'huxley building', 'blackett laboratory', 'skempton building', 'bessemer building',
    'roderic hill building', 'royal school of mines', 'city and guilds building', 'electrical engineering building',
    'dyson building', 'sir alexander fleming building', 'william penney laboratory', 'business school',
    '170 queen\'s gate', 'central library', 'imperial college union', 'beit quad', 'ethos sports centre',
    'garden hall', 'weeks hall', 'dangoor plaza', "queen's tower",
    # food and drink
    'pret', 'library cafe', 'rasa kitchen', 'bakery', 'college cafe', 'fuel']
LOCATION_ALIASES = {
    'restroom': 'bathroom', 'washroom': 'bathroom', 'toilets': 'bathroom',
    'lecture hall': 'lecture theatre', 'lecture room': 'lecture theatre', 'theatre': 'lecture theatre',
    'tutorial room': 'seminar room', 'lab': 'laboratory', 'computer lab': 'computer room', 'study area': 'study room',
    'lounge': 'living room', 'sitting room': 'living room', 'front room': 'living room', 'hall': 'hallway',
    'utility room': 'laundry room', 'back garden': 'garden', 'yard': 'garden',
    'sherfield': 'sherfield building', 'huxley': 'huxley building', 'blackett': 'blackett laboratory', 'blackett lab': 'blackett laboratory',
    'skempton': 'skempton building', 'bessemer': 'bessemer building', 'besmer': 'bessemer building', 'roderic hill': 'roderic hill building',
    'rsm': 'royal school of mines', 'city and guilds': 'city and guilds building',
    'electrical engineering': 'electrical engineering building', 'eee': 'electrical engineering building', 'triple e': 'electrical engineering building', 'eee building': 'electrical engineering building', 'triple e building': 'electrical engineering building',
    'dyson': 'dyson building', 'fleming': 'sir alexander fleming building',
    'alexander fleming building': 'sir alexander fleming building', 'penney': 'william penney laboratory', 'penney lab': 'william penney laboratory',
    '170 queens gate': '170 queen\'s gate', 'library': 'central library', 'union': 'imperial college union',
    'beit quadrangle': 'beit quad', 'ethos': 'ethos sports centre', 'queens tower': "queen's tower",
    'pret a manger': 'pret'}

# Objects the detector looks for: a copy of VOCABULARY, WORN and ALIASES in ros/src/autonomy/perception/object_detection.py.
# That file imports ROS and OpenCV, so it cannot be imported here. When its lists change, copy them again.
VOCABULARY = [
    # people and animals
    'person', 'dog', 'cat', 'bird', 'horse',
    # furniture and the building
    'chair', 'couch', 'bed', 'table', 'bench', 'door', 'door handle', 'window', 'stairs', 'elevator', 'light switch',
    'toilet', 'sink', 'bathtub', 'shower', 'mirror', 'lamp', 'clock', 'shelf', 'cupboard', 'bin', 'plant', 'vase',
    # electronics
    'tv', 'laptop', 'keyboard', 'mouse', 'remote', 'cell phone', 'headphones', 'charger', 'printer', 'speaker',
    # kitchen
    'refrigerator', 'microwave', 'oven', 'toaster', 'kettle', 'bottle', 'can', 'cup', 'bowl', 'plate', 'fork',
    'knife', 'spoon',
    # food
    'banana', 'apple', 'orange', 'sandwich', 'pizza', 'cake',
    # personal things
    'suitcase', 'keys', 'wallet', 'book', 'pen', 'scissors', 'toothbrush', 'towel', 'pillow', 'box', 'hair drier',
    # sport and play
    'sports ball', 'frisbee', 'tennis racket', 'skateboard',
    # street
    'traffic light', 'stop sign', 'fire hydrant', 'car', 'bus', 'truck', 'bicycle', 'motorcycle']
WORN = ['backpack', 'handbag', 'umbrella', 'glasses', 'hat', 'jacket', 'shoe', 'teddy bear']
OBJECTS = VOCABULARY + WORN
OBJECT_ALIASES = {
    'people': 'person', 'man': 'person', 'woman': 'person', 'child': 'person', 'human': 'person', 'puppy': 'dog',
    'kitten': 'cat',
    'seat': 'chair', 'armchair': 'chair', 'stool': 'chair', 'sofa': 'couch', 'settee': 'couch', 'desk': 'table',
    'dining table': 'table', 'doorway': 'door', 'handle': 'door handle', 'door knob': 'door handle',
    'doorknob': 'door handle', 'steps': 'stairs', 'staircase': 'stairs', 'lift': 'elevator', 'switch': 'light switch',
    'loo': 'toilet', 'bath': 'bathtub', 'washbasin': 'sink', 'basin': 'sink', 'light': 'lamp', 'bookshelf': 'shelf',
    'cabinet': 'cupboard', 'wardrobe': 'cupboard', 'trash can': 'bin', 'rubbish bin': 'bin',
    'potted plant': 'plant', 'flower pot': 'plant', 'flowers': 'vase',
    'television': 'tv', 'television set': 'tv', 'monitor': 'tv', 'screen': 'tv', 'computer screen': 'tv',
    'computer': 'laptop', 'remote control': 'remote', 'tv remote': 'remote', 'phone': 'cell phone',
    'mobile phone': 'cell phone', 'mobile': 'cell phone', 'smartphone': 'cell phone', 'earphones': 'headphones',
    'headset': 'headphones', 'cable': 'charger',
    'fridge': 'refrigerator', 'water bottle': 'bottle', 'mug': 'cup', 'glass': 'cup',
    'wine glass': 'cup', 'dish': 'plate',
    'rucksack': 'backpack', 'bag': 'handbag', 'purse': 'handbag', 'luggage': 'suitcase', 'key': 'keys',
    'sunglasses': 'glasses', 'spectacles': 'glasses', 'cap': 'hat', 'coat': 'jacket', 'shoes': 'shoe', 'trainers': 'shoe',
    'pencil': 'pen', 'cushion': 'pillow', 'package': 'box', 'parcel': 'box', 'hair dryer': 'hair drier',
    'ball': 'sports ball', 'football': 'sports ball', 'soccer ball': 'sports ball', 'basketball': 'sports ball',
    'tennis ball': 'sports ball', 'rugby ball': 'sports ball',
    'crossing light': 'traffic light', 'pedestrian light': 'traffic light', 'bike': 'bicycle', 'motorbike': 'motorcycle',
    'van': 'truck', 'lorry': 'truck'}

# "Take me to the toilet" means the room, not the fixture, when that room is a stored location.
GUIDE_TO_LOCATION = {'toilet': 'bathroom'}
# If Pete cannot find the object while guiding, try this stored location instead.
OBJECT_FALLBACK_LOCATIONS = {'toilet': 'bathroom', 'door': 'exit', 'refrigerator': 'kitchen', 'sink': 'kitchen'}
