<div align="center">

[![Banner SDC][banner]][link-repo]

[![Build Status][badge-build]][link-repo]
[![Python Version][badge-python]][link-repo]
[![License][badge-license]][link-repo]

</div>

<div align="center">
    <h1>Self Driving Challenge - Edition 2024</h1>
    <p>This repository contains the code used by the team of the Hanze during the Self Driving Challenge 2024</p>
</div>

## Table of Contents
- [About](#about)
- [The Challenges](#the-challenges)
- [Getting Started](#getting-started)
    - [Installation](#installation)
    - [Priming the Go-Kart](#priming-the-go-kart)
    - [Starting the Go-Kart](#starting-the-go-kart)
    - [Manual Driving](#manual-driving)
        - [Bluetooth Support](#bluetooth-support-linux)
        - [Controller Bindings](#controller-bindings)
    - [Scripts](#scripts)
- [License](#license)

## About
The Self Driving Challenge is an annual competition organized by the RDW, where teams from different universities and colleges in the Netherlands compete against each other in a series of challenges. The goal of the competition is to develop a self-driving car that can navigate through a series of challenges, such as following a line, avoiding obstacles, and recognizing traffic signs.

## The Challenges
### 1. Start on green traffic light

The vehicle starts in front of a traffic light. The first objective is to detect when the traffic light turns green. Once it has turned green, the vehicle may start driving.

### 2. Adhere to the speed limit

Traffic signs indicating speed limits will be placed along the track. Multiple different signs may be present. The vehicle should adhere to these limits, until a new speed-limit is given.

### 3. Traffic light stop and go

A red traffic light with a stop line will be present on the track. The vehicle approaching the traffic light will need to stop at an adequate location in front of the traffic light, and wait for it to turn green before continuing.

### 4. Pedestrian crossing

A pedestrian may be waiting on the side of a zebra crossing. The approaching vehicle will need to detect the presence of a waiting pedestrian, wait for them to cross and then continue its journey once the zebra crossing is cleared.

### 5. Overtaking manoeuvre

A stationary vehicle will be present in the lane of the vehicle. The vehicle must plan and execute an overtaking maneuver by changing lanes, passing the obstacle, and then returning to its original lane.

### 6. Parallel parking objective

To finish the parkour, the vehicle will need to perform a parallel parking manoeuvre. It will need to park itself in a parking spot surrounded by barriers.

## Getting Started
### Installation
**Prerequisites**

This project is built using Python. Ensure you have Python 3.12 and its package manager pip installed on your system. Verify their versions in your terminal with these commands:
```bash
python --version
pip --version
```

**Step 1: Clone the Repository**
```bash
git clone https://github.com/HeadTriXz/SDC-2024
```

**Step 2: Create a Virtual Environment**
```bash
python -m venv venv
```

**Step 3: Activate the Virtual Environment**
```bash
# On Windows
venv\Scripts\activate

# On macOS and Linux
source venv/bin/activate
```

**Step 4: Install the Required Packages**
```bash
pip install -r requirements.txt
```

### Priming the Go-Kart
Before starting the go-kart, let's ensure it is ready:
1. **Plug in the Intel NUC:**
Make sure the Intel NUC is securely connected to a power source and powered on.

2. **Connect the Controller:**
Connect the controller to the Intel NUC via USB or Bluetooth (see [Bluetooth Support](#bluetooth-support-linux)).

3. **Calibrate the Cameras:**
    - Place the ChArUco board in front of the go-kart.
    - Ensure the ChArUco board is clearly visible on all three cameras.
    - Run [the calibrate_cameras script](wiki-calibration) to calibrate the cameras: 
        ```bash
        python -m scripts.python.calibrate_cameras
        ```

### Starting the Go-Kart

To start the go-kart, run the following command:

```bash
python -m src.main
```

The vehicle will start in manual driving mode. To switch to autonomous driving mode, hold the **Start** or **Select** button on the controller. Switching to autonomous mode will be indicated by 3 short vibrations followed by one long vibration. Switching back to manual mode will be indicated by one long vibration.

### Manual Driving

Manual driving allows for direct control over the vehicle. This mode is essential for situations where human intervention is needed or for initial testing before deploying autonomous mode.

#### Bluetooth Support (Linux)

Bluetooth support is not enabled by default. To enable it, configure the following udev rule:

```bash
KERNEL=="event[0-9]*", SUBSYSTEM=="input", ATTRS{name}=="Xbox Wireless Controller", ACTION=="add", SYMLINK+="input/by-id/usb-Microsoft_Controller_Wireless-event-joystick"
```

#### Controller Bindings

The kart can be manually controlled using the following controller bindings:

| Input               | Type  | Action                                                                                                                                                                                                            |
|---------------------|-------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| Left joystick       |       | Controls the steering of the vehicle. Move left to steer left and right to steer right.                                                                                                                           |
| Left trigger        |       | Applies the brake. The further you press, the stronger the braking force.                                                                                                                                         |
| Right trigger       |       | Controls the throttle. The further you press, the faster the vehicle accelerates.                                                                                                                                 |
| A-button            | Hold  | Press and hold to enable the controller.                                                                                                                                                                          |
| B-button            | Press | Sets the gear to forward, allowing the vehicle to move forward.                                                                                                                                                   |
| X-button            | Press | Sets the gear to reverse, allowing the vehicle to move backwards.                                                                                                                                                 |
| Y-button            | Press | Sets the gear to neutral, stopping the vehicle from moving forward or backwards.                                                                                                                                  |
| Start/Select-button | Hold  | Switch between manual and autonomous mode. In autonomous mode, 3 short vibrations followed by one long vibration will indicate activation. Switching back to manual mode will be indicated by one long vibration. |

### Scripts

To execute a script, run the following command in your terminal:

```bash
python -m scripts.python.<script_name>
```

#### Available Scripts
- **[braking_calibration](scripts/python/braking_calibration.py)** - Calibrate maximum braking force using a binary search algorithm.
- **[calibrate_cameras](scripts/python/calibrate_cameras.py)** - Calibrate the cameras and generate the matrices required to generate a top-down view of the road.
- **[data_driving](scripts/python/data_driving.py)** - Capture images from the cameras while manually driving.
- **[view_lidar](scripts/python/view_lidar.py)** - Visualize lidar sensor data for analysis and debugging.

## License
This project is licensed under the **MIT License**.

See the [LICENSE](LICENSE) file for more information.

<!-- Badges -->
[badge-build]:https://img.shields.io/github/actions/workflow/status/HeadTriXz/SDC-2024/ruff.yml?branch=main&style=for-the-badge
[badge-license]:https://img.shields.io/badge/license-MIT-blue.svg?style=for-the-badge
[badge-python]:https://shields.io/badge/python-3.10_|_3.11_|_3.12-blue?style=for-the-badge

<!-- Links -->
[link-repo]:https://github.com/HeadTriXz/SDC-2024
[wiki-calibration]:https://github.com/HeadTriXz/SDC-2024/wiki/Camera-Calibration
[banner]:https://github.com/HeadTriXz/SDC-2024/assets/32986761/e1194707-aa35-4649-b31d-97624179e18f
