from __future__ import print_function
import time
from dronekit import connect, VehicleMode, LocationGlobalRelative
from pymavlink import mavutil
import numpy as np
import cv2
import params
import cv2.aruco as aruco
import math

# local modules
import params
from camera_helper import USBVideoStream, FPS

def dist(x,y,z):
    return math.sqrt(x**2+y**2+z**2)

vehicle = connect('/dev/ttyACM0', baud = 921600, wait_ready = True)

# 定义arm_and_takeoff函数，使无人机解锁并起飞到目标高度
# 参数aTargetAltitude即为目标高度，单位为米

def arm_and_takeoff(aTargetAltitude):

# 进行起飞前检查
    '''
    print("Basic pre-arm checks")
    
    # vehicle.is_armable会检查飞控是否启动完成、有无GPS fix、卡曼滤波器

    while not vehicle.is_armable:
        print(" Waiting for vehicle to initialise...")        
        time.sleep(1)    

    # 解锁无人机（电机将开始旋转）

    print("Arming motors")
    '''
    # 将无人机的飞行模式切换成"GUIDED"（一般建议在GUIDED模式下控制无人机）

    vehicle.mode = VehicleMode("GUIDED")
  
    # 通过设置vehicle.armed状态变量为True，解锁无人机
    
    vehicle.armed = True

    

    # 在无人机起飞之前，确认电机已经解锁
    '''
    while not vehicle.armed:        
        print(" Waiting for arming...")        
        time.sleep(1)
    '''

    # 发送起飞指令
    
    print("Taking off!")
    
    # simple_takeoff将发送指令，使无人机起飞并上升到目标高度


    vehicle.simple_takeoff(aTargetAltitude)

    

    # 在无人机上升到目标高度之前，阻塞程序
    
    while True:
        
        print(" Altitude: ", vehicle.location.global_relative_frame.alt)
        
        # 当高度上升到目标高度的0.95倍时，即认为达到了目标高度，退出循环

        # vehicle.location.global_relative_frame.alt为相对于home点的高度

        time.sleep(2)
        '''if vehicle.location.global_relative_frame.alt >= aTargetAltitude * 0.95:
 
            print("Reached target altitude")'''

        break

        # 等待1s

        time.sleep(1)

# 无人机速度设定函数
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

# 无人机航向设定
def condition_yaw(self,heading,relative,clock_wise):
    if relative:
        isRelative = 1
    else:
        isRelative = 0

    if clock_wise:
        direction = 1
    else:
        direction = -1

    if not relative:
        direction = 0

    msg = self.vehicle.message_factory.command_long_encode(
        0,0,
        mavutil.mavlink.MAV_CMD_CONDITION_YAW,
        0,
        heading,
        0,
        direction,
        isRelative,
        0,0,0)

    self.vihecle.send_mavlink(msg)
    self.vehicle.flush()

# 起飞以及目标高度设定
arm_and_takeoff(10)


# 设置在运动时的空速(读取时表示当前空速)
vehicle.airspeed = 3


# 发送指令，让无人机前往目标点位
point1 = LocationGlobalRelative(31.026546,121.422065,2)

vehicle.simple_goto(point1)
# 这里还需要改，因为没有弄明白如何让无人机到达指定点后停止！！！
time.sleep(5)
'''
while(vehicle.location.global_frame != 31.026546,121.422065,2):
        vehicle.simple_goto(point1)
'''

# 到达指定点后的操作
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

        #到达指定高度
        '''vehicle.wait_for_alt(1)'''
        '''
        #原地转圈圈找tag
        while ids is None:
            condition_yaw(vehicle,90,1,1)
        '''
        # 这里应该添加寻找tag的动作！！！
        if ids is not None:
            rvecs, tvecs, _objPoints = aruco.estimatePoseSingleMarkers(corners, 0.15, params.mtx, params.dist)
            prompt_strings = []
            for rvec, tvec in zip(rvecs, tvecs):
                print("rvec ", rvec)
                print("tvec ", tvec)
                frame = aruco.drawAxis(frame, params.mtx, params.dist, rvec, tvec, 0.15)
                prompt_strings.append("x:{:.2f} y:{:.2f} z:{:.2f}".format(*(tvec[0])))
                #prompt_strings.append("x:{:.2f} y:{:.2f} z:{:.2f}".format(*(rvec[0])))
            cv2.putText(frame, '\n'.join(prompt_strings), (25,25), cv2.FONT_HERSHEY_SIMPLEX, 1, (255,255,255), 1)
        
            '''# 无人机的动作，先随便写一下！！！
            while abs(tvec[0]+0.3)>0.2 or abs(tvec[2]-0.8)>0.2:
                if tvec[0]+0.3>0.2:
                    send_nav_velocity(vehicle,0,0.5,0)
                    time.sleep(abs(tvec[0]+0.3))
                    break
                elif tvec[0]+0.3<-0.2:
                    send_nav_velocity(vehicle,0,-0.5,0)			
                    time.sleep(abs(tvec[0]+0.3))
                    break
                if tvec[2]-0.8>0.2:
                    send_nav_velocity(vehicle,0,0,-0.5)
                    time.sleep(abs(tvec[2]-0.8))
                    break
                elif tvec[2]-0.8<-0.2:
                    send_nav_velocity(vehicle,0,0,0.5)
                    time.sleep(abs(tvec[2]-0.8))
                    break'''
        cv2.imshow('frame', frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
        if ids == 0:
            break
        

    #降落
    #send_nav_velocity(vehicle,-1,0,0)
    #time.sleep(5)

    #重新起飞，返航，先这样吧，不行再调（又是一个大坑）！！！
    #arm_and_takeoff(2)
    

    #vehicle.mode = VehicleMode("RTL")

    print("emmm...")
    print(ids)	
    vehicle.armed = False
    vehicle.close()

    # When everything done, stop the stream
    stream.stop()
    print("time elapsed(s)", fps_counter.elapsed())
    print("average fps", fps_counter.fps())
    cv2.destroyAllWindows()
