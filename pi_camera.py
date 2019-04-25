#!/usr/bin/env python3
# import the necessary packages
from imutils.video import FPS
from picamera.array import PiRGBArray
from picamera import PiCamera
import params
import argparse
import imutils
import time
import cv2
import cv2.aruco as aruco
 
# initialize the camera and stream
camera = PiCamera()
camera.resolution = (640, 480)
camera.framerate = 32
rawCapture = PiRGBArray(camera, size=(640, 480))
stream = camera.capture_continuous(rawCapture, format="bgr",
          use_video_port=True)

print("sampling will start in 1 seconds...")
time.sleep(1.0)
fps = FPS().start()
print("sampling started.")

aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)

for i, f in enumerate(stream):
    frame = f.array
    # frame = imutils.resize(frame, width=400)

    corners, ids, rejecttedImgPoints = aruco.detectMarkers(frame, aruco_dict)
    frame = aruco.drawDetectedMarkers(frame, corners, ids)
    
    if ids is not None:    
        rvecs, tvecs, _objPoints = aruco.estimatePoseSingleMarkers(corners, 0.125, params.mtx, params.dist)
        for rvec, tvec in zip(rvecs, tvecs):
            frame = aruco.drawAxis(frame, params.mtx, params.dist, rvec, tvec, 0.125) 
    
    cv2.imshow("Frame", frame)
    key = cv2.waitKey(1)
    
    rawCapture.truncate(0)
    fps.update()

fps.stop()
print("elasped time: " + str(fps.elapsed()))
print("approx. FPS: " + str(fps.fps()))

cv2.destroyAllWindows()
stream.close()
rawCapture.close()
camera.close()

