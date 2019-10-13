#!/usr/bin/env python3

# test_link_cyj.py
# Test mavlink to px4 drone via usb serial
# @Author eyek(cyj205@sjtu.edu.cn)

import dronekit
import time
import termcolor
import logging

# print helper function
def print_var(a, b):
    print(termcolor.colored(a, 'green'), str(b), sep='\t')

def print_sep(title = ''):
    termcolor.cprint(title.center(50, '-'), 'yellow')

# log helper function
def print_log(msg):
    print('[', termcolor.colored(time.strftime('%H:%M:%S'), 'yellow'), ']', msg, sep='')

def print_status(vehicle):
    print_sep('vehicle status')
    print_var("Autopilot Firmware version", vehicle.version)
    print_var("Autopilot capabilities (supports ftp)", vehicle.capabilities.ftp)
    print_var("Global Location", vehicle.location.global_frame)
    print_var("Global Location (relative altitude)", vehicle.location.global_relative_frame)
    print_var("Local Location", vehicle.location.local_frame)
    print_var("Attitude", vehicle.attitude)
    print_var("Velocity", vehicle.velocity)
    print_var("GPS", vehicle.gps_0)
    print_var("Groundspeed", vehicle.groundspeed)
    print_var("Airspeed", vehicle.airspeed)
    print_var("Gimbal status", vehicle.gimbal)
    print_var("Battery", vehicle.battery)
    print_var("EKF OK?", vehicle.ekf_ok)
    print_var("Last Heartbeat", vehicle.last_heartbeat)
    print_var("Rangefinder", vehicle.rangefinder)
    print_var("Rangefinder distance", vehicle.rangefinder.distance)
    print_var("Rangefinder voltage", vehicle.rangefinder.voltage)
    print_var("Heading", vehicle.heading)
    print_var("Is Armable?", vehicle.is_armable)
    print_var("System status", vehicle.system_status.state)
    print_var("Mode", vehicle.mode.name)    # settable
    print_var("Armed", vehicle.armed)    # settable
    print_sep()

def take_off(vehicle, target_altitude):
    print_log("Taking off to {}m".format(target_altitude))
    vehicle.simple_takeoff(target_altitude)

    # block the program till vehicle reach desired altitude
    # while True:
        # print(" Altitude: ", vehicle.location.global_relative_frame.alt)
        # print_log("Current altitude: {}".format(vehicle.location.global_relative_frame.alt))
        # if vehicle.location.global_relative_frame.alt >= target_altitude * 0.95:
        #     print_log("Reached target altitude")
        #     break
    #    time.sleep(1)
    time.sleep(5)

def fly_to_location(vehicle, lat, lng, rel_alt = None):
    if rel_alt is None:
        print_log("Warning: rel_alt not defined!")
        print_log("Using default value 5 meters above starting position.")
        rel_alt = 5
    print_log("Flying to {:.5f},{:.5f},{:.2f}(rel)", lat, lng, rel_alt)
    vehicle.mode = dronekit.VehicleMode("GUIDED")
    location = LocationGlobalRelative(lat, lng, rel_alt)


def send_ned_velocity(vehicle, velocity, duration, freq=1):
    """
    Move vehicle in direction based on specified velocity vectors.
    """
    print_log("Moving vehicle, velocity=({:.2f}, {:.2f}, {:.2f})".format(*velocity))
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,       # time_boot_ms (not used)
        0, 0,    # target system, target component
        dronekit.mavutil.mavlink.MAV_FRAME_LOCAL_NED, # frame
        0b0000111111000111, # type_mask (only speeds enabled)
        0, 0, 0, # x, y, z positions (not used)
        *velocity,  # x, y, z velocity in m/s
        0, 0, 0, # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
        0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)

    # keep sending this message for `duration` seconds
    for x in range(0, int(duration * freq)):
        vehicle.send_mavlink(msg)
        time.sleep(1 / freq)

# disable dronekit logging first
print_log("Warning: Annoying \"wx index out of bounds\" value is disabled. Mind this if you want to see log from dronekit.")
dronekit_logger = logging.getLogger("autopilot")
dronekit_logger.disabled = True

# connect to vehicle
vehicle = dronekit.connect('/dev/ttyACM0', wait_ready=True, baud=921600)
print_log("Connection established.")
time.sleep(1)
print_status(vehicle)

try:
    # arm the vehicle
    print_log("Trying to arm vehicle...")
    vehicle.arm(timeout=5)

    # set to guided mode first
    vehicle.mode = dronekit.VehicleMode("GUIDED")

    # take off
    take_off(vehicle, 20)

    # set velocity
    # fly_to_location(vehicle, 31.0246941285, 121.4266920090)
    send_ned_velocity(vehicle, (5, 0, 0),  2)
    send_ned_velocity(vehicle, (-5, 0, 0), 2)
    send_ned_velocity(vehicle, (0, 5, 0),  2)
    send_ned_velocity(vehicle, (0, -5, 0), 2)
    send_ned_velocity(vehicle, (0, 0, 5),  2)
    send_ned_velocity(vehicle, (0, 0, -5), 2)

finally:
    # wait for 2 seconds and then disarm the vehicle
    vehicle.disarm(timeout=5)
    print_log("Vehicle disarmed")
