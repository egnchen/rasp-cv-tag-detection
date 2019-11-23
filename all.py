import numpy as np
import cv2
import params
import cv2.aruco as aruco
import time
import math

# local modules
import params
from camera_helper import USBVideoStream, PicVideoStream, FPS

from mavsdk import System
from mavsdk import OffboardError, VelocityBodyYawspeed
import asyncio


async def arm(drone):
    print("Arming and enabling offboard control...")
    try:
        await drone.action.arm()
        await drone.offboard.set_velocity_body(VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0))
        await drone.offboard.start()
    except OffboardError as error:
        printf("Error when trying to arm & enable offboard control. Disarming.")
        await drone.action.disarm()

async def takeoff(drone):
    print("Taking off")
    await drone.offboard.set_velocity_body(VelocityBodyYawspeed(0.0, 0.0, -0.5, 0.0))
    await asyncio.sleep(5)
    print("Stablizing...")
    await drone.offboard.set_velocity_body(VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0))

async def land(drone):
    print("Landing...")
    await drone.action.land()

if __name__ == "__main__":
    # initialize aruco detection
    aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_5X5_1000)
    # open video stream
    stream = PicVideoStream("/home/hongji-li/cam_plane", "default_typhoon_h480_cgo3_camera_link_camera(1)-", sync=True).start()
    
    # open mavsdk connection
    loop = asyncio.get_event_loop()
    drone = System()
    loop.run_until_complete(drone.connect(system_address="udp://:14550"))
    loop.run_until_complete(arm(drone))
    loop.run_until_complete(takeoff(drone))

    while True:
        # Capture frame-by-frame
        ret, frame = stream.read()
        # print("fps", stream.get_fps())

        corners, ids, rejecttedImgPoints = aruco.detectMarkers(frame, aruco_dict)
        frame = aruco.drawDetectedMarkers(frame, corners, ids)

        if ids is not None:    
            rvecs, tvecs = aruco.estimatePoseSingleMarkers(corners, 0.15, params.mtx, params.dist)
            print(rvecs, tvecs)
            prompt_strings = []
            for rvec, tvec in zip(rvecs, tvecs):
                frame = aruco.drawAxis(frame, params.mtx, params.dist, rvec, tvec, 0.15)
                prompt_strings.append("x:{:.2f} y:{:.2f} z:{:.2f}".format(*(tvec[0])))
                prompt_strings.append("x:{:.2f} y:{:.2f} z:{:.2f}".format(*(rvec[0])))
            cv2.putText(frame, '\n'.join(prompt_strings), (25,25), cv2.FONT_HERSHEY_SIMPLEX, 1, (255,255,255), 1)
        # Display the resulting frame
        cv2.imshow('frame', frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break


    # When everything done, stop the stream
    loop.run_until_complete(land(drone))
    stream.stop()
    print("average fps", stream.get_fps())
    cv2.destroyAllWindows()
