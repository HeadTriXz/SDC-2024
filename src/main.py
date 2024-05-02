import os

from src.kart import Kart

if __name__ == "__main__":
    if "ENVIRONMENT" in os.environ and os.environ["ENVIRONMENT"] == "simulator":
        from src.simulation.main import start_simulator

        start_simulator()
    else:
        Kart().start()

