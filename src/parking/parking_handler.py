# import time
#
# from src.driving.speed_controller import ISpeedController, SpeedControllerState
# from src.utils.lidar.ILidar import ILidar
#
#
# class ParkingHandler:
#     """A class that handles the parking"""
#
#     def __init__(self, lidar: ILidar, speed_controller: ISpeedController) -> None:
#         """Create a parking handler"""
#         self.__lidar = lidar
#         self.__speed_controller = speed_controller
#         self.__can_controller = speed_controller.can_controller
#
#     def start_parking(self) -> None:
#         """Start the parking"""
#         time.sleep(3)
#         self.__speed_controller.state = SpeedControllerState.DRIVING
#         self.__speed_controller.target_speed = 3
#         distance = 7
#         wall_detected = False
#
#         while True:
#             # lidar_data = client.getLidarData()
#             system = client.getDistanceSensorData("right")
#             # print("INITIAL DISTANCE: ", system.distance)
#             if not wall_detected and system.distance <= distance + 0.4:
#                 print("WALL DETECTED")
#                 distance = system.distance
#                 wall_detected = True
#
#             if wall_detected and system.distance >= distance + 2:
#                 print("WALL PASSED, gap detected")
#                 break
#             time.sleep(0.1)
#
#         self.__can_controller.steering = 1.0
#         distance = 100
#         while True:
#             distancefront = client.getDistanceSensorData("front").distance
#             if distancefront < distance:
#                 distance = distancefront
#                 if distance < 50 and distancefront > 14:
#                     break
#             time.sleep(0.1)
#
#         print("TURNING")
#         print("DISTANCE: ", distance)
#
#         while True:
#             print(distance)
#             distances = client.getDistanceSensorData("left")
#             print(distances.distance)
#             if distances.distance < distance / 2 + 2:
#                 self.__can_controller.steering = -1.0
#                 break
#             time.sleep(0.1)
#
#         while True:
#             distances = client.getDistanceSensorData("front")
#             if distances.distance < 3:
#                 self.__speed_controller.state = SpeedControllerState.STOPPED
#                 break
#             time.sleep(0.1)
