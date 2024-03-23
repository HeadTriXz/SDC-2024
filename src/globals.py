# TODO: convert to proper application state management
GLOBALS = {
    "REQUESTED_LANE": 0,
    "SET_SPEED": 5,
    "GAMMA": {
        "LEFT": 1,
        "CENTER": 1,
        "RIGHT": 1,
        "ADJUST": False,
    },
    "LANE_DETECTION": {
        "LINE_WIDTH": 50,
        "ZEBRA_CROSSING_THRESHOLD": 20_000,
        "LINE_THRESHOLD": 2_500,
        "PIXELS_IN_WINDOW": 1,
    },
    "WHITE": {
        "MIN": 200,
        "MAX": 255,
    },
}
