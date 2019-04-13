# Tag detection
## Part of the drone delivery platform - a college student innovation program project.

This repo implements the visual positioning & controlling functionalities for the drone delivery platform.

In brief, the program detects the aruco tag captured by the PiCam, calculates the relative position between it and the drone, and use mavlink protocol to navigate PixHawk-based drone to land on the tag safely.

## TODO List

- [ ] Generation of Aruco tags
- [x] Detection of Aruco tags
- [ ] Positioning of Aruco tags in 3D space
- [ ] Communication through mavlink protocol
- [ ] PID remote drone controlling
- [ ] Extra demands & functionalities emerge during development

## Getting started

### Development

Prerequisities:

* Python 3.x
    * OpenCV python binding(with contribution library)
    * Dronekit
    * Mavlink

To install them on a raspberry pi:
```bash
sudo apt install python3
pip3 install opencv-contrib-python
pip3 install dronekit
```

Installing on windows follows similiar procedures. You'll have to `pip3 uninstall opencv-python` first if you already have the opencv python binding installed without contribution libraries, otherwise the two modules will conflict.

After cloning the repository, run `aruco_detect.py` to begin tag detection.
