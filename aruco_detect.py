import numpy as np
import cv2
from . import params
import cv2.aruco as aruco

if __name__ == "__main__":
    cap = cv2.VideoCapture(0)
    aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
    
    while(True):
        # Capture frame-by-frame
        ret, frame = cap.read()

        corners, ids, rejecttedImgPoints = aruco.detectMarkers(frame, aruco_dict)
        frame = aruco.drawDetectedMarkers(frame, corners, ids)

        # Display the resulting frame
        cv2.imshow('frame', frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # When everything done, release the capture
    cap.release()
    cv2.destroyAllWindows()
