import rospy
import numpy as np
from std_msgs.msg import Float32MultiArray as fl

import os
import sys
from dynamixel_sdk import *

# 현재 스크립트의 디렉토리 경로를 가져옵니다.
module_directory = os.path.join(os.path.dirname(__file__), "DynamixelSDK/ros/dynamixel_sdk")
# sys.path 리스트에 모듈 디렉토리를 추가합니다.
sys.path.append(module_directory)

DEVICENAME = '/dev/ttyUSB0'

# XC330-T288 Control table address and protocol version
XC_ADDR_TORQUE_ENABLE = 64
XC_ADDR_GOAL_POSITION = 116
XC_ADDR_PRESENT_POSITION = 132

XC_PROTOCOL_VERSION = 2.0

XC_DXL_ID = 4
XC_BAUDRATE = 115200

XC_TORQUE_ENABLE = 1
XC_TORQUE_DISABLE = 0

class DynamixelNode:
    def __init__(self):
        self.port_handler_xc = PortHandler(DEVICENAME)
        self.packet_handler_xc = PacketHandler(XC_PROTOCOL_VERSION)

        # Open port
        if self.port_handler_xc.openPort():
            rospy.loginfo("Successfully opened the XC port.")
        else:
            rospy.logerr("Failed to open the XC port.")
            rospy.signal_shutdown("Failed to open the XC port.")
            return

        # Set baud rate
        if self.port_handler_xc.setBaudRate(XC_BAUDRATE):
            rospy.loginfo("Successfully set the XC baudrate.")
        else:
            rospy.logerr("Failed to set the XC baudrate.")
            rospy.signal_shutdown("Failed to set the XC baudrate.")
            return

        # Enable torque
        dxl_comm_result, dxl_error = self.packet_handler_xc.write1ByteTxRx(self.port_handler_xc, XC_DXL_ID, XC_ADDR_TORQUE_ENABLE, XC_TORQUE_ENABLE)
        if dxl_comm_result != COMM_SUCCESS:
            rospy.logerr(f"Failed to enable torque for XC Motor ID: {XC_DXL_ID}")
            rospy.signal_shutdown(f"Failed to enable torque for XC Motor ID: {XC_DXL_ID}")
            return
        else:
            rospy.loginfo(f"Torque enabled for XC Motor ID: {XC_DXL_ID}")

    def gripper(self, desired_angle_deg):
        desired_position = int(desired_angle_deg * 11.378)  # Convert degrees to Dynamixel units

        # Write goal position
        dxl_comm_result, dxl_error = self.packet_handler_xc.write4ByteTxRx(self.port_handler_xc, XC_DXL_ID, XC_ADDR_GOAL_POSITION, desired_position)
        if dxl_comm_result != COMM_SUCCESS:
            rospy.logerr(f"Failed to write position for XC Motor ID: {XC_DXL_ID}")
        else:
            rospy.loginfo(f"Moved XC Motor ID: {XC_DXL_ID} to position: {desired_position}")

    def shutdown(self):
        # Disable torque and close port on shutdown
        self.packet_handler_xc.write1ByteTxRx(self.port_handler_xc, XC_DXL_ID, XC_ADDR_TORQUE_ENABLE, XC_TORQUE_DISABLE)
        self.port_handler_xc.closePort()
        rospy.loginfo("Shutdown Dynamixel node.")

def move(initial_position, goal_position):
    pub_initial_position = rospy.Publisher('initial_position', fl, queue_size=10)
    pub_goal_position = rospy.Publisher('goal_position', fl, queue_size=10)
    rate = rospy.Rate(10)

    # Publish initial position
    initial_msg = fl()
    initial_msg.data = initial_position
    pub_initial_position.publish(initial_msg)
    rospy.loginfo("Initial position is %.2f %.2f %.2f", *initial_position)

    rate.sleep()

    # Publish goal position
    goal_msg = fl()
    goal_msg.data = goal_position
    pub_goal_position.publish(goal_msg)
    rospy.loginfo("Goal position is %.2f %.2f %.2f", *goal_position)

    rate.sleep()
    rospy.loginfo("Publishing finished for gripper")

def main():
    rospy.init_node('pick_and_place', anonymous=True)

    dynamixel_node = DynamixelNode()
    rospy.on_shutdown(dynamixel_node.shutdown)

    l1 = (80) + 40 + 2 # mm
    d1 = np.deg2rad(45 + 7)  # degree
    P1 = [l1 * np.sin(d1), l1 * np.cos(d1)]

    l2 = (30) + 40 + 15   # mm
    d2 = np.deg2rad(180 -50)  # degree
    P2 = [l2 * np.sin(d2), l2 * np.cos(d2)]

    l3 = 27
    P3 = [l3 * np.sin(d2), l3 * np.cos(d2)]

    Dg = 97
    Dh = 200

    A = [P1[0], P1[1], Dg]
    B = [P1[0], P1[1], Dh]
    C = [[P2[0]+P3[0]*0.0, P2[1]+P3[1]*0.0, Dh], [P2[0]+P3[0]*1.0, P2[1]+P3[1]*1.0, Dh],[P2[0]+P3[0]*2.0, P2[1]+P3[1]*2.0, Dh],[P2[0]+P3[0]*3.0, P2[1]+P3[1]*3.0, Dh],
        [P2[0]+P3[0]*0.5, P2[1]+P3[1]*0.5, Dh], [P2[0]+P3[0]*1.5, P2[1]+P3[1]*1.5, Dh],[P2[0]+P3[0]*2.5, P2[1]+P3[1]*2.5, Dh],
        [P2[0]+P3[0]*1.0, P2[1]+P3[1]*1.0, Dh], [P2[0]+P3[0]*2.0, P2[1]+P3[1]*2.0, Dh],
        [P2[0]+P3[0]*1.5, P2[1]+P3[1]*1.5, Dh+20]]
    D = [[P2[0]+P3[0]*0.0, P2[1]+P3[1]*0.0, Dg+25*0], [P2[0]+P3[0]*1.0, P2[1]+P3[1]*1.0, Dg+25*0],[P2[0]+P3[0]*2.0, P2[1]+P3[1]*2.0, Dg+25*0],[P2[0]+P3[0]*3.0, P2[1]+P3[1]*3.0, Dg+25*0],
        [P2[0]+P3[0]*0.5, P2[1]+P3[1]*0.5, Dg+25*1], [P2[0]+P3[0]*1.5, P2[1]+P3[1]*1.5, Dg+25*1],[P2[0]+P3[0]*2.5, P2[1]+P3[1]*2.5, Dg+25*1],
        [P2[0]+P3[0]*1.0, P2[1]+P3[1]*1.0, Dg+25*2], [P2[0]+P3[0]*2.0, P2[1]+P3[1]*2.0, Dg+25*2],
        [P2[0]+P3[0]*1.5, P2[1]+P3[1]*1.5, Dg+25*3]]


    Open = 340
    Close = 285

    move(B, B)
    rospy.sleep(2)
    dynamixel_node.gripper(Open)

    for i in range(10):
        rospy.sleep(5)
        move(B, A)
        rospy.sleep(4)
        dynamixel_node.gripper(Close)
        move(A, B)
        rospy.sleep(3)
        move(B, C[i])
        move(C[i], D[i])
        rospy.sleep(7)
        dynamixel_node.gripper(Open)
        rospy.sleep(1)
        move(D[i], C[i])
        move(C[i], B)
        rospy.sleep(2)

if __name__ == '__main__':
    try:
        rospy.logwarn("pick_and_place Node is on")
        main()
    except rospy.ROSInterruptException:
        rospy.loginfo('Program is shut down')
