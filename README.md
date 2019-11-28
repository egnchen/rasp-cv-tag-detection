# Tag detection
## Part of the drone delivery platform - a college student innovation program project.

This repo implements the visual positioning & controlling functionalities for the drone delivery platform.

In brief, the program detects the aruco tag captured by the PiCam, calculates the relative position between it and the drone, and use mavlink protocol to navigate PixHawk-based drone to land on the tag safely.

## TODO List

- [ ] Generation of Aruco tags(this is emitted, just use some tools)
- [x] Detection of Aruco tags
- [x] Positioning of Aruco tags in 3D space
	- [x] Pose estimation
	- [x] Distance estimation(only linear distance now)
- [x] Communication through mavlink protocol
- [x] PID remote drone controlling
- [ ] Extra demands & functionalities emerge during development

## Getting started

### Development

Prerequisities:

* Python 3.x
    * OpenCV python binding(with contribution library)
    * MAVSDK python binding

To install them on a raspberry pi:
``` bash
sudo apt install python3
pip3 install opencv-contrib-python
```

**MAVSDK installation**: Python wheels of `MAVSDK-Python` on raspberry pi platform is currently not available. This gives extra complexity to deploying MAVSDK onto Raspberry Pi platform.

Basically, you'll have to clone the `MAVSDK-Python` repository from github and install the pip package manually. You'll also have to compile the **MAVSDK backend**(`mavsdk_server` process) from the MAVSDK C++ main repository and move it along side the Python binding libraries so that it can be invoked. Refer to [Mavsdk-python github](https://github.com/mavlink/MAVSDK-Python#build-and-run-from-sources) and cross compiling backend page from MAVSDK [documentation](https://mavsdk.mavlink.io/develop/en/contributing/build.html) to get step-by-step instructions.


Installing on windows follows similiar procedures. You'll have to `pip3 uninstall opencv-python` first if you already have the opencv python binding installed without contribution libraries, otherwise the two modules will conflict.

After cloning the repository, run `aruco_detect.py` to begin tag detection.

### Tag detection

Run `aruco_detect.py` in a Python 3 interpreter to run the detection program. It uses `cv2.VideoCapture()`, so it should work fine on both laptop camerars and usb camera on a Raspberry Pi.
