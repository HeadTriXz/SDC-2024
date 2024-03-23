from constants import SpeedMode


speed_mode: SpeedMode = SpeedMode.SLOW
speed_mode_to_speed = {SpeedMode.SLOW: 25, SpeedMode.MEDIUM: 50, SpeedMode.FAST: 75, SpeedMode.VERY_FAST: 100}

speed = speed_mode_to_speed[speed_mode]
