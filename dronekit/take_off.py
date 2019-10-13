from __future__ import print_function
import time
from dronekit import connect, VehicleMode, LocationGlobalRelative
from pymavlink import mavutil

vehicle = connect('/dev/ttyACM0', baud = 921600, wait_ready = True)

def send_nav_velocity(self,v_x,v_y,v_z):	
    msg =self.vehicle.message_factory.set_position_target_local_ned_encode(
        0,
        0,
        0,
        mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
        0b0000111111000111,
        0, 0, 0,
        v_x, v_y, v_z,
        0, 0, 0,
        0, 0)

    self.vihecle.send_mavlink(msg)
    self.vehicle.flush()

vehicle.mode = VehicleMode("GUIDED")
vehicle.armed = True

vehicle.simple_takeoff(1)
while True:
	if vehicle.location.global_relative_frame.alt >= 0.95:
		break

time.sleep(2)

send_nav_velocity(vehicle,-1,0,0)
time.sleep(1)

vehicle.armed = False
vehicle.close()
