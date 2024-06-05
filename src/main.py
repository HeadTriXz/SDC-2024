import logging
import os

from src.driving.kart import Kart
from src.telemetry.logging_handler import LoggingHandler


if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO, handlers=[LoggingHandler()])

    if "ENVIRONMENT" in os.environ and os.environ["ENVIRONMENT"] == "simulator":
        from src.simulation.main import start_simulator

        start_simulator()
    else:
        Kart().start()


