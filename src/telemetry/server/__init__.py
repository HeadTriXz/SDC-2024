from typing import Any

import uvicorn
from fastapi import FastAPI, Form, Request, Response

from globals import GLOBALS
from lane_assist import LineFollowing
from telemetry.server.utils import get_file_relative_path

app = FastAPI()

_line_follower: LineFollowing | None = None


@app.get("/")
async def read_root(request: Request):
    """Get the root page."""
    # load the page from resources
    with open("../resources/web_content/html/index.html") as file:
        return Response(content=file.read(), media_type="text/html")


@app.get("/pid")
async def read_pid(request: Request):
    """Get the pid values."""
    if _line_follower is None:
        return {
            "kp": 0,
            "ki": 0,
            "kd": 0,
        }

    return {
        "kp": _line_follower.pid.Kp,
        "ki": _line_follower.pid.Ki,
        "kd": _line_follower.pid.Kd,
    }


@app.post("/pid")
def update_pid(kp: float = Form(...), ki: float = Form(...), kd: float = Form(...)):
    """Update the pid values."""
    if _line_follower is not None:
        _line_follower.pid.kp = kp
        _line_follower.pid.ki = ki
        _line_follower.pid.kd = kd

    # redirect to the home page
    return Response(status_code=303, headers={"Location": "/"})


@app.get("/line_following")
async def read_thresholds(request: Request):
    return GLOBALS["LANE_DETECTION"]


@app.post("/line_following")
def update_thresholds(
    line_width: int = Form(...),
    zebra_crossing_threshold: int = Form(...),
    line_threshold: int = Form(...),
    pixels_in_window: int = Form(...),
):
    GLOBALS["LANE_DETECTION"] = {
        "LINE_WIDTH": line_width,
        "ZEBRA_CROSSING_THRESHOLD": zebra_crossing_threshold,
        "LINE_THRESHOLD": line_threshold,
        "PIXELS_IN_WINDOW": pixels_in_window,
    }

    # redirect to the home page
    return Response(status_code=303, headers={"Location": "/"})


## DONE


@app.get("/speed")
def get_speed() -> Any:
    return {"speed": GLOBALS["SET_SPEED"]}


@app.post("/speed")
def set_speed(speed: int = Form(...)) -> Any:
    GLOBALS["SET_SPEED"] = speed
    return Response(status_code=303, headers={"Location": "/"})


@app.get("/lane")
def get_lane() -> Any:
    return {"lane": GLOBALS["REQUESTED_LANE"]}


@app.post("/lane")
def set_lane(lane: int = Form(...)) -> Any:
    GLOBALS["REQUESTED_LANE"] = lane
    return Response(status_code=303, headers={"Location": "/"})


@app.get("/gamma")
def get_gamma() -> Any:
    return GLOBALS["GAMMA"]


@app.post("/gamma")
def set_gamma(
    left: float = Form(...), center: float = Form(...), right: float = Form(...), gamma: bool = Form(...)
) -> Any:
    GLOBALS["GAMMA"] = {
        "LEFT": left,
        "CENTER": center,
        "RIGHT": right,
        "ADJUST": gamma,
    }
    return Response(status_code=303, headers={"Location": "/"})


def run(line_follower: LineFollowing) -> None:
    """Run the webserver server."""
    global _line_follower
    _line_follower = line_follower

    uvicorn.run(app, host="0.0.0.0", port=8000)


if __name__ == "__main__":
    run(None)
