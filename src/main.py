import logging
import os

from src.driving.can import CANController, get_can_bus
from src.driving.modes import AutonomousDriving
from src.telemetry.logging_handler import LoggingHandler


if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO, handlers=[LoggingHandler()])

    if "ENVIRONMENT" in os.environ and os.environ["ENVIRONMENT"] == "simulator":
        from src.simulation.main import start_simulator

        start_simulator()
    else:
        bus = get_can_bus()
        can = CANController(bus)
        can.start()

        kart = AutonomousDriving(can)
        kart.toggle()
        kart.start()
