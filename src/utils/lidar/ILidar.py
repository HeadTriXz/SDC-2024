
from abc import ABC, abstractmethod
from threading import Thread


class ILidar(ABC):

    def __init__(self) -> None:
        self.thread = Thread(target=self.capture, daemon=True)

    @abstractmethod
    def capture(self) -> None:
        """A function that captures the data from the lidar and filters it."""
        pass

    def start(self) -> None:
        """Start the lidar."""
        self.running = True
        if not self.thread.is_alive():
            self.thread.start()

    def stop(self) -> None:
        """Stop the lidar."""
        self.running = False
