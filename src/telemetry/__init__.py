import threading

from lane_assist.line_following.path_follower import PathFollower
from telemetry.server import run


def start_telemetry(line_follower: PathFollower) -> None:
    """Start the telemetry server."""
    thread = threading.Thread(target=run, args=(line_follower,), daemon=True)
    thread.start()
