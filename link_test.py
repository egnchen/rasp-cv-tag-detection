#!/usr/bin/env python

# -*- coding: utf-8 -*-
from dronekit import connect, VehicleMode, LocationGlobalRelative

from pymavlink import mavutil

vehicle = connect('/dev/ttyACM0', baud = 921600, wait_ready = True)

def arm_and_takeoff(aTargetAltitude):
    
	print("Basic pre-arm checks")
	'''while not vehicle.is_armable:
		print(" Waiting for vehicle to initialise...")
		time.sleep(1)
	print("Arming motors")'''
	vehicle.mode = VehicleMode("GUIDED")    
	vehicle.armed = True
	   
	'''while not vehicle.armed:
		print(" Waiting for arming...")
		time.sleep(1)'''

	print("Taking off!")
	vehicle.simple_takeoff(aTargetAltitude)
	while True:
		print(" Altitude: ", vehicle.location.global_relative_frame.alt)
		if vehicle.location.global_relative_frame.alt >= aTargetAltitude * 0.95:
			print("Reached target altitude")
			break
			time.sleep(1)


'''def send_nav_velocity(self,v_x,v_y,v_z):
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
'''


arm_and_takeoff(0.3)
time.sleep(10)

vehicle.armed = false
'''print("Set default/target airspeed to 3")


vehicle.airspeed = 3



point1 = LocationGlobalRelative(31.026546,121.422065,2)

# point2 = LocationGlobalRelative()

# point3 = LocationGlobalRelative()

# point4 = LocationGlobalRelative()

# point5 = LocationGlobalRelative()


vehicle.simple_goto(point1)


time.sleep(5)


if ids is not None:
	vehicle.wait_for_alt(1)
	
	while abs(tvec[0]+0.3)>0.2||abs(tvec[2]-0.8)>0.2:
		if tvec[0]+0.3>0.2:
			send_nav_velocity(,0,1,0)
			time.sleep(abs(tvec[0]+0.3))
		elif tvec[0]+0.3<-0.2:
			send_nav_velocity(,0,-1,0)			
			time.sleep(abs(tvec[0]+0.3))
		if tvec[2]-0.8>0.2:
			send_nav_velocity(,0,0,-1)
			time.sleep(abs(tvec[2]-0.8))
		elif tvec[2]-0.8<-0.2:
			send_nav_velocity(,0,0,1)
			time.sleep(abs(tvec[2]-0.8))

send_nav_velocity(,-1,0,0)
time.sleep(15)



#vehicle.mode = VehicleMode("RTL")
'''

print("Close vehicle object")

vehicle.close()
