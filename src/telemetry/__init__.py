from lane_assist import LineFollowing
from telemetry.server import run


def start(line_follower: LineFollowing) -> None:
    """Start the telemetry server."""
    run(line_follower)
