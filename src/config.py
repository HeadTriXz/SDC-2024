from constants import SpeedMode

speed_mode: SpeedMode = SpeedMode.SLOW
speed_mode_to_speed = {SpeedMode.SLOW: 25, SpeedMode.MEDIUM: 50, SpeedMode.FAST: 75, SpeedMode.VERY_FAST: 100}

speed = speed_mode_to_speed[speed_mode]

requested_lane = 0
requested_speed = 5

gamma = {"LEFT": 1, "CENTER": 1, "RIGHT": 1, "ADJUST": False}
lane_detection = {"LINE_WIDTH": 50, "ZEBRA_CROSSING_THRESHOLD": 15_000, "LINE_THRESHOLD": 1000, "PIXELS_IN_WINDOW": 1}
white = {"MIN": 185, "MAX": 255}

camera_ids = {"left": 0, "center": 0, "right": 0}
