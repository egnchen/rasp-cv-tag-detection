import numpy as np
import cv2
import params
import cv2.aruco as aruco
import time
import math

# local modules
import params
from camera_helper import USBVideoStream, FPS


def dist(x,y,z):
    return math.sqrt(x**2+y**2+z**2)


if __name__ == "__main__":
    aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
    fps_counter = FPS()
    stream = USBVideoStream().start()
    fps_counter.start()

    while(fps_counter.frames() < 5000):
        # Capture frame-by-frame
        ret, frame = stream.read()
        fps_counter.update()
        print(fps_counter.currentFps())

        corners, ids, rejecttedImgPoints = aruco.detectMarkers(frame, aruco_dict)
        frame = aruco.drawDetectedMarkers(frame, corners, ids)

        if ids is not None:    
            rvecs, tvecs, _objPoints = aruco.estimatePoseSingleMarkers(corners, 0.15, params.mtx, params.dist)
            prompt_strings = []
            for rvec, tvec in zip(rvecs, tvecs):
                print("rvec ", rvec)
                print("tvec ", tvec)
                frame = aruco.drawAxis(frame, params.mtx, params.dist, rvec, tvec, 0.15)
                prompt_strings.append("x:{:.2f} y:{:.2f} z:{:.2f}".format(*(tvec[0])))
                # prompt_strings.append("x:{:.2f} y:{:.2f} z:{:.2f}".format(*(rvec[0])))
            cv2.putText(frame, '\n'.join(prompt_strings), (25,25), cv2.FONT_HERSHEY_SIMPLEX, 1, (255,255,255), 1)
        # Display the resulting frame
        cv2.imshow('frame', frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # When everything done, stop the stream
    stream.stop()
    print("time elapsed(s)", fps_counter.elapsed())
    print("average fps", fps_counter.fps())
    cv2.destroyAllWindows()
