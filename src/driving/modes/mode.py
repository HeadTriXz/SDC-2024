from abc import ABC, abstractmethod


class DrivingMode(ABC):
    """An abstract class for a driving mode."""

    @abstractmethod
    def start(self) -> None:
        """Start the driving mode."""
        pass

    @abstractmethod
    def toggle(self) -> None:
        """Toggle the driving mode."""
        pass
