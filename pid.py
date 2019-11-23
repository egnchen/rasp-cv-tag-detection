import numpy as np
import cv2
import params
import cv2.aruco as aruco
import time
import math
import asyncio
from mavsdk import System
from mavsdk import OffboardError, VelocityBodyYawspeed

# local modules
import params
from camera_helper import USBVideoStream, PicVideoStream, FPS

async def take_off():
    drone = System()
    await drone.connect(system_address="udp://:14540")
    print("Waiting for drone...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print(f"Drone discovered with UUID: {state.uuid}")
            break

    print("-- Arming")
    await drone.action.arm()

    print("-- Setting initial setpoint")
    await drone.offboard.set_velocity_body(VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0))

    print("-- Starting offboard")
    try:
        await drone.offboard.start()
    except OffboardError as error:
        print(f"Starting offboard mode errored with code {error._result.result}")
        print("-- Disarming")
        await drone.action.disarm()
        return

    print("-- Taking off")
    await drone.offboard.set_velocity_body(VelocityBodyYawspeed(0.0, 0.0, -0.5, 0.0))
    await asyncio.sleep(3)
    await drone.offboard.set_velocity_body(VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0))

async def landing():
    drone = System()
    await drone.connect(system_address="udp://:14540")
    print("-- Landing")
    await drone.action.land()

async def set_v(x, y, z, yaw,t):
    drone = System()
    await drone.connect(system_address="udp://:14540")
    await drone.offboard.set_velocity_body(VelocityBodyYawspeed(x, y, z, yaw))
    await asyncio.sleep(t)


if __name__ == "__main__":
    aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_5X5_1000)
    stream = PicVideoStream("/home/hongji-li/cam_plane", "default_typhoon_h480_cgo3_camera_link_camera(1)-", sync=True).start()

    loop = asyncio.get_event_loop()
    loop.run_until_complete(take_off())
    v_p = 0.3
    v_i = 0.02
    v_d = 0.05
    y_p = 0.4
    y_i = 0
    y_d = 0

    x_pro = 0
    y_pro = 0
    z_pro = 0.5
    yaw_pro = 0.5
    x_int = 0
    y_int = 0
    z_int = 0
    yaw_int = 0
    x_diff = 0
    y_diff = 0
    z_diff = 0
    yaw_diff = 0
    x_set = 0
    y_set = 0
    z_set = 0
    yaw_set = 0.5
    i = 0
    flag = False

    while True:
        # Capture frame-by-frame
        ret, frame = stream.read()

        corners, ids, rejecttedImgPoints = aruco.detectMarkers(frame, aruco_dict)
        frame = aruco.drawDetectedMarkers(frame, corners, ids)

        if ids is None:
            i += 1
            x_set = x_pro/5
            y_set = 0.0
            z_set = z_pro/5
            yaw_set = yaw_pro/5
            t = 0.5
            if i == 8 and flag:
                loop.run_until_complete(set_v(0.0, 0.0, 0.0, 0.0,1))
                loop.run_until_complete(landing())
                break
            elif i == 30:
                loop.run_until_complete(set_v(0.0, 0.0, 0.0, 0.0,1))
                loop.run_until_complete(landing())
                print("1111")
                break

        else:
            flag = True
            i = 0
            rvecs, tvecs = aruco.estimatePoseSingleMarkers(corners, 0.15, params.mtx, params.dist)

            x_diff = tvecs[0][0][0] - x_pro + 1.0
            y_diff = tvecs[0][0][1] - y_pro + 0.8
            z_diff = tvecs[0][0][2] - z_pro - 1
            yaw_diff = rvecs[0][0][2] - yaw_pro
            x_pro = tvecs[0][0][0] + 1.0
            y_pro = tvecs[0][0][1] + 0.8
            z_pro = tvecs[0][0][2] - 1
            yaw_pro = rvecs[0][0][2]
            x_int += x_pro
            y_int += y_pro
            z_int += z_pro
            yaw_int += yaw_pro

            x_set = x_pro*v_p + x_int*v_i + x_diff*v_d
            y_set = y_pro*v_p + y_int*v_i + y_diff*v_d
            z_set = z_pro*v_p + z_int*v_i + z_diff*v_d
            yaw_set = yaw_pro*y_p + yaw_int*y_i + yaw_diff*y_d
            t = 0.5
            if x_pro**2 + y_pro**2 + z_pro**2 < 0.3:
                loop.run_until_complete(set_v(0.0, 0.0, 0.0, 0.0,1))
                loop.run_until_complete(landing())
                break

        loop.run_until_complete(set_v(z_set, x_set, -y_set, yaw_set,t))

    stream.stop()
    print("average fps", stream.get_fps())
    cv2.destroyAllWindows()

