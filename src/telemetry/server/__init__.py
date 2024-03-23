# ruff: noqa: ERA001, D103, ANN201, A002, ARG001
# we ignore a large amount of linting errors because this file will hopefully be removed soon.
from typing import Any

import uvicorn
from fastapi import FastAPI, Form, Request, Response

import config
from lane_assist import PathFollower
from telemetry.server.utils import get_file_relative_path

app = FastAPI()

_line_follower: PathFollower | None = None


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
        _line_follower.pid.Kp = kp
        _line_follower.pid.Ki = ki
        _line_follower.pid.Kd = kd

    # redirect to the home page
    return Response(status_code=303, headers={"Location": "/"})


@app.get("/line_following")
async def read_thresholds(request: Request):
    return config.lane_detection


@app.post("/line_following")
def update_thresholds(
    line_width: int = Form(...),
    zebra_crossing_threshold: int = Form(...),
    line_threshold: int = Form(...),
    pixels_in_window: int = Form(...),
):
    config.lane_detection = {
        "LINE_WIDTH": line_width,
        "ZEBRA_CROSSING_THRESHOLD": zebra_crossing_threshold,
        "LINE_THRESHOLD": line_threshold,
        "PIXELS_IN_WINDOW": pixels_in_window,
    }

    # redirect to the home page
    return Response(status_code=303, headers={"Location": "/"})


@app.get("/speed")
def get_speed():
    return {"speed": config.requested_speed}


@app.post("/speed")
def requested_speed(speed: int = Form(...)):
    config.requested_speed = speed
    return Response(status_code=303, headers={"Location": "/"})


@app.get("/lane")
def get_lane():
    return {"lane": config.requested_lane}


@app.post("/lane")
def set_lane(lane: int = Form(...)):
    config.requested_lane = lane
    return Response(status_code=303, headers={"Location": "/"})


@app.get("/gamma")
def get_gamma():
    return config.gamma


@app.post("/gamma")
def set_gamma(left: float = Form(...), center: float = Form(...), right: float = Form(...), gamma: bool = Form(...)):
    config.gamma = {
        "LEFT": left,
        "CENTER": center,
        "RIGHT": right,
        "ADJUST": gamma,
    }
    return Response(status_code=303, headers={"Location": "/"})


@app.get("/white")
def get_white():
    return config.white


@app.post("/white")
def set_white(min: int = Form(...), max: int = Form(...)):
    config.white = {
        "MIN": min,
        "MAX": max,
    }
    return Response(status_code=303, headers={"Location": "/"})


def run(line_follower: PathFollower) -> None:
    """Run the webserver server."""
    global _line_follower
    _line_follower = line_follower

    uvicorn.run(app, host="0.0.0.0", port=8000)


if __name__ == "__main__":
    run(None)
