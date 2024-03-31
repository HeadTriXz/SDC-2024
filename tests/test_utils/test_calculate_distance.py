import unittest

from utils.calculate_distance import meters_to_y, y_to_meters


class TestCalculateDistance(unittest.TestCase):
    """Tests for the calculate_distance module."""

    def test_y_to_meters(self) -> None:
        """Test the y_to_meters function."""
        self.assertEqual(y_to_meters(1080, 1080), y_to_meters(720, 720))
        self.assertEqual(y_to_meters(561, 1080), y_to_meters(374, 720))

    def test_meters_to_y(self) -> None:
        """Test the meters_to_y function."""
        self.assertEqual(meters_to_y(y_to_meters(1080, 1080), 1080), 1080)
        self.assertEqual(meters_to_y(y_to_meters(720, 720), 720), 720)
        self.assertEqual(meters_to_y(y_to_meters(561, 1080), 1080), 561)


if __name__ == "__main__":
    unittest.main()
