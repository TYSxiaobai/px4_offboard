#!/usr/bin/env python
# -*- coding:utf-8 -*-
import socket
import struct
import rospy
from mavros_msgs.msg import State, PositionTarget
from geometry_msgs.msg import PoseStamped
from mavros_msgs.srv import CommandBool, CommandTOL
from mavros_msgs.srv import SetMode
import time
#from simple_pid import PID
import threading
import numpy as np
import math
import tf
#import serial#串口包
from std_msgs.msg import Float64
###############################################################################################

# find_fire = False#去年做的火警检测，所以这个变量我目前设置成fire
# fire_over = False
# fire_flag = False
foundation_flag = False
develop_flag = False
take_off = False

index_list = []

mission_height = 1.2#起飞高度

now_yaw = 0.0
# file_path = "/home/nx/chongshan_ws/src/chongshan_offboard/route/wonderful.txt"
file_path = "/home/nx/yuanshen-drone/src/realflight_modules/px4ctrl/route/test1.txt"#这个是路径文件
                #文件路径替换成你的路径文件txt的路径
                #路径格式为：
                #x轴飞行位置     y轴飞行位置     z轴飞行位置     停留时间/s
                #例如：
                #1.0 1.0 1.5 1.0
                #飞完最后一个点会自动降落
                #方向：前方x，左方y，上方z

#pico = serial.Serial("/dev/ttypico", 115200)#pico串口通信

# class PositionPacket:
#     def __init__(self, x_pose, y_pose, fire):
#         self.x_pose = x_pose
#         self.y_pose = y_pose
#         self.fire = fire

#     def pack(self):
#         packed_data = struct.pack('!ff?', self.x_pose, self.y_pose, self.fire)
#         return packed_data

###############################################################################################
    
def pose(x, y, z):
    global set_yaw
    set_target_local = PositionTarget()
    set_target_local.type_mask=0b100111111000
    set_target_local.coordinate_frame=1
    set_target_local.position.x=x
    set_target_local.position.y=y
    set_target_local.position.z=z
    set_target_local.yaw=set_yaw
    return set_target_local

current_state = State()

###############################################################################################

def state_cb(msg):
    global current_state
    current_state = msg

current_pos= PoseStamped()

###############################################################################################

def  pos_cb(msg):
    global current_pos, now_yaw
    current_pos = msg
    now_yaw = tf.transformations.euler_from_quaternion([msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w])[2]
    
################################################################################################
        
def waypoint_release(point_x, point_y, point_z):#目标点飞行函数
    global current_pos, incremental_x, incremental_y, deviation_x, deviation_y
    
    while not rospy.is_shutdown():
        if judge_position(current_pos.pose.position.x, point_x) or judge_position(current_pos.pose.position.y, point_y) or judge_position(current_pos.pose.position.z, point_z):
            local_target_pub.publish(pose(point_x, point_y, point_z))
            print("x:%f, y:%f, z:%f, x__:%f, y__:%f, z__:%f" %(current_pos.pose.position.x,current_pos.pose.position.y,current_pos.pose.position.z, point_x, point_y, point_z))
        else:
            #print("Arrive!")
            break
        rate.sleep()
##############################################################################################

def judge_position(pos, target):##目标点飞行误差容忍值，来判断当前飞行的位置是否为目标点区域内
    if pos <= target+0.06 and pos >= target-0.06:
        return False 
    else:
        return True


####################################################################################################################################################################################################

if __name__ == "__main__":
    rospy.init_node("multi_uav")
    # 设置参数默认值
    rospy.get_param('/xy_coordinates/x', 0.0)
    rospy.get_param('/xy_coordinates/y', 0.0)
    incremental_x = 0.0 
    incremental_y = 0.0
    deviation_x = 0.0 
    deviation_y = 0.0 
    count = 0
    rospy.Subscriber("/mavros/state", State, state_cb, queue_size=10)
    rospy.Subscriber("/mavros/local_position/pose", PoseStamped, pos_cb, queue_size=10)
    armServer = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
    takeoffServer = rospy.ServiceProxy('/mavros/cmd/takeoff', CommandTOL)
    setModeServer = rospy.ServiceProxy('/mavros/set_mode', SetMode)
    local_target_pub = rospy.Publisher('/mavros/setpoint_raw/local', PositionTarget, queue_size=10)
    #local_target_pub = rospy.Publisher('mavros/setpoint_position/local', PositionTarget, queue_size=10)
    
    time.sleep(10)
    rate = rospy.Rate(10.0)

    with open(file_path, 'r') as f:
        established_route = f.readlines()
        print(established_route)
        
    set_yaw = now_yaw

    print('take off!!!')
    time.sleep(3)
  
    while not rospy.is_shutdown():
        if current_state.mode != "OFFBOARD" :
            setModeServer(custom_mode='OFFBOARD')#offboard模式切换，如果第一次检测到飞机不是offboard模式，它会自动切换成offboard，后面可以手动切出
            local_target_pub.publish(pose(0, 0, mission_height))
            #print("Offboard enabled")
            # print("Wait switch to Offboard")
        else:
            if not current_state.armed:
                armServer(True)
                print("Vehicle armed")
            elif current_pos.pose.position.z <= mission_height - 0.2:
                #waypoint_release(0, 0, misssion_height)
                local_target_pub.publish(pose(0, 0, mission_height))
                print(current_pos.pose.position.z)
                print("Take off")
            else:
                break
        rate.sleep()
        
    time.sleep(2)
    print("start")
    while not rospy.is_shutdown():
        for route in established_route:
            print(route)
            point = route.split()
            waypoint_release(float(point[0]), float(point[1]), float(point[2]))
            time.sleep(float(point[3]))
        break

    setModeServer(custom_mode='AUTO.LAND')
    print("降落成功")
    time.sleep(1)
