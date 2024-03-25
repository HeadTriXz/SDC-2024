from constants import SpeedMode, Label

speed_mode: SpeedMode = SpeedMode.SLOW
speed_mode_to_speed = {SpeedMode.SLOW: 25, SpeedMode.MEDIUM: 50, SpeedMode.FAST: 75, SpeedMode.VERY_FAST: 100}

speed = speed_mode_to_speed[speed_mode]

# Object Detection
model_path = "../../resources/models/best.pt"

class_to_speed = {
    Label.SPEED_LIMIT_5: 5,
    Label.SPEED_LIMIT_10: 10,
    Label.SPEED_LIMIT_15: 15,
    Label.SPEED_LIMIT_20: 20,
    Label.SPEED_LIMIT_30: 30,
    Label.SPEED_LIMIT_40: 40
}

# # Pedestrian Detection
crosswalk_overlap_margin = 0.1
crosswalk_safe_zone_margin = 0.1
crosswalk_min_distance = 2
