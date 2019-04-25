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
print("Module initialization completed.")

# initialize the camera and stream
camera = PiCamera()
camera.resolution = (640, 480)
camera.framerate = 32
rawCapture = PiRGBArray(camera, size=camera.resolution)
stream = camera.capture_continuous(rawCapture, format="bgr",
          use_video_port=True)
print("Camera stream initialization completed.")
print("Sampling will start in 1 second...")
time.sleep(1.0)
fps = FPS().start()
print("Sampling started.")

aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)

for i, f in enumerate(stream):
    frame = f.array
    # frame = imutils.resize(frame, width=400)

    corners, ids, rejecttedImgPoints = aruco.detectMarkers(frame, aruco_dict)
    frame = aruco.drawDetectedMarkers(frame, corners, ids)
    
    if ids is not None:    
        rvecs, tvecs, _objPoints = aruco.estimatePoseSingleMarkers(corners, 0.15, params.mtx, params.dist)
        prompt_strings = []
        for rvec, tvec in zip(rvecs, tvecs):
            frame = aruco.drawAxis(frame, params.mtx, params.dist, rvec, tvec, 0.15)
            prompt_strings.append("x:{:.2f} y:{:.2f} z:{:.2f}".format(*(tvec[0])))
        cv2.putText(frame, '\n'.join(prompt_strings), (25,25), cv2.FONT_HERSHEY_SIMPLEX, 1, (255,255,255), 1)
    
    cv2.imshow("Frame", frame)
    key = cv2.waitKey(1) & 0xFF
    if key == ord('q'):
        break
    
    rawCapture.truncate(0)
    fps.update()

fps.stop()
print("elasped time: " + str(fps.elapsed()))
print("approx. FPS: " + str(fps.fps()))

cv2.destroyAllWindows()
stream.close()
rawCapture.close()
camera.close()

