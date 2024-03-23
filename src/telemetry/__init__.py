import threading

from lane_assist import LineFollowing
from telemetry.server import run


def start_telemetry(line_follower: LineFollowing) -> None:
    """Start the telemetry server."""
    thread = threading.Thread(target=run, args=(line_follower,), daemon=True)
    thread.start()
