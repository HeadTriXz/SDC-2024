> Wow! Interfaces. I love interfaces. They are so useful. I can't believe I didn't use them before. I'm going to use them all the time now because they are so useful and I love them. I love interfaces. I love them so much that I'm going to use them all the time.
> 
> GitHub Copilot <3

```py
from abc import ABC, abstractmethod

"""
Controllers
"""
class ObjectController:
    handlers: list[ObjectHandler]
    lane_controller: LaneController
    speed_controller: SpeedController

    def __init__(self):
        self.handlers = []
    
    def add_handler(self, handler):
        self.handlers.append(handler)
    
    def handle(self, obj):
        for handler in self.handlers:
            handler.handle(obj)

    def set_max_speed(self, speed: int):
        self.speed_controller.max_speed = speed

    def set_stopped(self, stopped: bool):
        self.speed_controller.stopped = stopped

class LaneController:
    speed_controller: SpeedController

    def get_current_lane(self):
        pass

    def get_lanes(self):
        pass

    def set_lane(self, idx: int):
        pass

class SpeedController:
    max_speed: int
    target_speed: int
    stopped: bool

    def get_current_speed(self):
        pass

"""
Object Handlers
"""
class ObjectHandler(ABC):
    def __init__(self, controller: ObjectController):
        self.controller = controller
    
    @abstractmethod
    def handle(self, obj):
        pass

class SpeedSignHandler(ObjectHandler):
    def handle(self, obj):
        self.controller.set_max_speed(obj.speed)

class TrafficLightHandler(ObjectHandler):
    def handle(self, obj):
        self.controller.set_stopped(True)

class ObstacleHandler(ObjectHandler):
    def handle(self, obj):
        pass

class PedestrianHandler(ObjectHandler):
    def handle(self, obj):
        pass
```
