class Window:
    """Class to represent a window in the image."""

    def __init__(self, x: int, y: int, margin: int) -> None:
        """Initialize the window."""
        self.x = x
        self.y = y
        self.margin = margin

        self.collided = False
        self.found_in_previous = False
