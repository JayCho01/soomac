## angle -> move motort / angle(0,0,0,0) -> end point will be located in (396, 0, 102.5)

import os
import sys
import rospy
import time
import numpy as np
# 현재 스크립트의 디렉토리 경로를 가져옵니다.
#module_directory = os.path.join(os.path.dirname(__file__), "~/DynamixelSDK/ros/dynamixel_sdk")
module_directory = os.path.expanduser("~/DynamixelSDK/ros/dynamixel_sdk/src")
# module_directory = os.path.join(os.path.expanduser("~"), "DynamixelSDK/ros/dynamixel_sdk")

# sys.path 리스트에 모듈 디렉토리를 추가합니다.
sys.path.append(module_directory)

from dynamixel_sdk import *
from dynamixel_sdk.port_handler import PortHandler
from dynamixel_sdk.packet_handler import PacketHandler
from dynamixel_sdk.robotis_def import *

from std_msgs.msg import Float32MultiArray as fl
from std_msgs.msg import Float32, Bool
# import threading
# import matplotlib.pyplot as plt
# import matplotlib.animation as animation
# import plotly.graph_objects as go
DEVICENAME = '/dev/ttyUSB1'


#**********************XM430-W350-R(PROTOCOL_VERSION 2.0)**************************#

# Control table address
XM_ADDR_TORQUE_ENABLE           = 64
XM_ADDR_VELOCITY_I_GAIN         = 76
XM_ADDR_VELOCITY_P_GAIN         = 78
XM_ADDR_POTISION_D_GAIN         = 80
XM_ADDR_POSITION_I_GAIN         = 82
XM_ADDR_POSITION_P_GAIN         = 84
XM_ADDR_FEEDFORWARD_2ND_GAIN    = 88
XM_ADDR_FEEDFORWARD_1ST_GAIN    = 90
XM_ADDR_PROFILE_ACCELERATION    = 108
XM_ADDR_PROFILE_VELOCITY        = 112
XM_ADDR_GOAL_POSITION           = 116
XM_ADDR_MOVING                  = 122
XM_ADDR_MOVING_STATUS           = 123
XM_ADDR_PRESENT_POSITION        = 132
XM_ADDR_VELOCITY_LIMIT          = 44

XM_CURRENT_ADDR = 126

XM_PROTOCOL_VERSION = 2.0

XM_DXL_ID = [0, 1, 2, 3, 4]
gripper_DXL_ID = 5
XM_BAUDRATE = 115200

XM_TORQUE_ENABLE = 1
XM_TORQUE_DISABLE = 0

#xm_packet_handler = PacketHandler(XM_PROTOCOL_VERSION)

# 각 모터에 대한 ID 및 레지스터 주소 설정
#XM_DXL_ADDR_PRESENT_POSITION = 126  # XM 모터의 현재 위치 값을 읽기 위한 레지스터 주소


################################################################################################################################################
# parameter
N = 50 # link motor 경로 분활
N_grip = 50 # gripper motor 경로 분활
timer = time.time()
repeat_time = 0.05
impact_num = 13



class Impact: # 작업 예정
    def __init__(self):
        self.last_torgue = np.array([])
        self.diff_torques = np.array([]).reshape(0,4)
        self.last_diff_2rd = np.array([])

    def diff(self, current_torque): # 입력 : 현재 토크 값, 출력은 없으며 전역 변수 diff_torques에 토크 변화량이 저장됨
        # print("#######미분#######")
        if len(self.last_torgue)!=0: # last_torque 존재 시   
            diff_torque = self.last_torgue - current_torque
            self.last_torgue = current_torque
            self.diff_torques = np.append(self.diff_torques,[np.abs(diff_torque)], axis = 0)
        else:
            self.last_torgue = current_torque

        if self.diff_torques.shape[0]>=10: # 10개 이상의 토크 미분값이 저장된 경우
            self.diff_torques[0] = self.diff_torques[8]
            self.diff_torques[1] = self.diff_torques[9]
            self.diff_torques = self.diff_torques[:2]

    def impact_check(self):
        #print("#######충격 확인#######")
        diff_torques_num = self.diff_torques.shape[0] # 저장된 변화량의 개수
        dof = self.diff_torques.shape[1] # 모터의 개수(4)
        diff_2rd = np.zeros(dof) # 토크 변화량의 기울기가 급변할 때를 파악하기 위함.
        result = 0
        if self.diff_torques.shape[0] >= 2:
            for i in range(dof):
                diff_2rd[i] = self.diff_torques[diff_torques_num-1][i]-self.diff_torques[diff_torques_num-2][i]
            diff_2rd = np.abs(diff_2rd)
            # print("diff_2rd : ", diff_2rd)

            if len(self.last_diff_2rd)!=0:
                key = 0
                for i in range(dof):
                    if abs(diff_2rd[i])>=0.1 and abs(self.last_diff_2rd[i])>=0.1: 
                        if abs(diff_2rd[i]-self.last_diff_2rd[i])>impact_num*min(abs(diff_2rd[i]), abs(self.last_diff_2rd[i])):
                            key = key + 1 
                if key >= 1:
                    rospy.loginfo("impact")
                    result = 1     
                    print("diff_2rd : ", diff_2rd)
                    print("last_diff_2rd : ",self.last_diff_2rd)
                    
                # else:
                #     #print("Non-impact")
                    
                #     # print("diff_2rd : ", diff_2rd)
                #     # print("last_diff_2rd : ",self.last_diff_2rd)
                self.last_diff_2rd = diff_2rd
            
            else:
                print("Non-data")
                self.last_diff_2rd = diff_2rd
        return result
            
    def print_info(self, test_data):
        print("input data : ",test_data)
        print("last_torque : ",self.last_torgue)
        print("diff_torques : ",self.diff_torques)
        print("last_diff_2rd : ",self.last_diff_2rd)


class DynamixelNode:
    def __init__(self): # 파라미터 설정
        # setting
        self.dynamixel_setting()
        self.impact_check = Impact()
        self.change_para = 1024/90

        # init_setting
        self.gripper_open = 1800
        self.gripper_close = 3100

        self.gripper_open_mm = 72 #54
        self.gripper_close_mm = 15.9 #0
        self.current_grip_seperation = self.gripper_open 
        self.seperation_per_mm = (self.gripper_close/(self.gripper_open_mm-self.gripper_close_mm))
        
        self.packet_handler_xm.write4ByteTxRx(self.port_handler_xm, gripper_DXL_ID, XM_ADDR_GOAL_POSITION, self.current_grip_seperation) #실행 시 그리퍼 open

        ## ROS
        # for state done
        self.state_finish = rospy.Publisher('state_done', Bool, queue_size=10)

        # for plot_torque
        self.pub_data_1 = rospy.Publisher('XM_0', Float32, queue_size=10)
        self.pub_data_2 = rospy.Publisher('XM_1', Float32, queue_size=10)
        self.pub_data_3 = rospy.Publisher('XM_2', Float32, queue_size=10)
        self.pub_data_4 = rospy.Publisher('XM_3', Float32, queue_size=10)
        self.pub_data_5 = rospy.Publisher('XM_4', Float32, queue_size=10)

        # self.data_array = np.zeros((20, 5))  # (20, 4) 형태의 데이터 배열 초기화

    def dynamixel_setting(self): # 다이나믹셀 세팅
        # XM 모터와 통신을 위한 포트 핸들러 및 패킷 핸들러 초기화
        self.port_handler_xm = PortHandler(DEVICENAME)
        self.packet_handler_xm = PacketHandler(XM_PROTOCOL_VERSION)

        # XM 모터 포트 열기
        if self.port_handler_xm.openPort():
            rospy.loginfo("Successfully opened the XM port.")
        else:
            rospy.logerr("Failed to open the XM port.")
            rospy.signal_shutdown("Failed to open the XM port.")
            return

        # XM 모터 통신 속도 설정
        if self.port_handler_xm.setBaudRate(XM_BAUDRATE):
            rospy.loginfo("Successfully set the XM baudrate.")
        else:
            rospy.logerr("Failed to set the XM baudrate.")
            rospy.signal_shutdown("Failed to set the XM baudrate.")
            return
        
        # link 모터의 토크 활성화
        for dxl_id in XM_DXL_ID:
            dxl_comm_result, dxl_error = self.packet_handler_xm.write1ByteTxRx(self.port_handler_xm, dxl_id, XM_ADDR_TORQUE_ENABLE, XM_TORQUE_ENABLE)
            dxl_comm_result, dxl_error = self.packet_handler_xm.write1ByteTxRx(self.port_handler_xm, dxl_id, XM_ADDR_PROFILE_ACCELERATION, 5)

            if dxl_comm_result != COMM_SUCCESS:
                rospy.logerr("Failed to enable torque for XM Motor ID: {}".format(dxl_id))
                rospy.signal_shutdown("Failed to enable torque for XM Motor ID: {}".format(dxl_id))
                return
            else:
                rospy.loginfo("Torque enabled for XM Motor ID: {}".format(dxl_id))
        
        # gripper 모터의 토크 활성화
        dxl_comm_result, dxl_error = self.packet_handler_xm.write1ByteTxRx(self.port_handler_xm, gripper_DXL_ID, XM_ADDR_TORQUE_ENABLE, XM_TORQUE_ENABLE)
        dxl_comm_result, dxl_error = self.packet_handler_xm.write1ByteTxRx(self.port_handler_xm, gripper_DXL_ID, XM_ADDR_PROFILE_ACCELERATION, 5)
        if dxl_comm_result != COMM_SUCCESS:
            rospy.logerr("Failed to enable torque for XM Motor ID: {}".format(gripper_DXL_ID))
            rospy.signal_shutdown("Failed to enable torque for XM Motor ID: {}".format(gripper_DXL_ID))
            return
        else:
            rospy.loginfo("Torque enabled for XM Motor ID: {}".format(gripper_DXL_ID))        
        
        
        # XM540 p gain 설정
        self.packet_handler_xm.write4ByteTxRx(self.port_handler_xm, 1, 84, 300) 

    def present_position(self, port_handler, packet_handler, dxl_id, addr_present_position): # 현재 모터 value 도출해주는 메서드
        # 모터의 현재 위치 읽기
        dxl_present_position, _, _ = packet_handler.read2ByteTxRx(port_handler, dxl_id, addr_present_position)
        return dxl_present_position

    def link(self, data): # 현재 각도 읽고, 목표 각도 까지 trajectory 만들어서 제어, input : 목표 각도(출발 각도는 받지 않아도 모터 자체에서 현재 각도 확인 후 traj)
        position_dynamixel = np.array(data.data)

        position_dynamixel[0] = 1007 # base 수리 되면 변경 -> (position_dynamixel[0]+180)*self.change_para
        position_dynamixel[1] = (position_dynamixel[1]+90)*self.change_para
        position_dynamixel[2] = (position_dynamixel[2]+180+20)*self.change_para
        position_dynamixel[3] = (position_dynamixel[3]+180)*self.change_para
        position_dynamixel[4] = (position_dynamixel[4]+180)*self.change_para        
        position_dynamixel = position_dynamixel.astype(int)
        # print(position_dynamixel)        
        present_position = np.zeros(5)

        for dxl_id in XM_DXL_ID:
            present_position[dxl_id] = self.present_position(self.port_handler_xm, self.packet_handler_xm, dxl_id, XM_ADDR_PRESENT_POSITION)

        # print(present_position)

        traj_arr = cubic_trajectory(present_position, position_dynamixel) # 각 모터마다의 각도를 trajectory, traj_arr는 (5,N)의 shape
        position_dynamixel = traj_arr        
        position_dynamixel = position_dynamixel.astype(int)
        # print(position_dynamixel)

        for n in range(N):
            for dxl_id in XM_DXL_ID:
                # print("Set Goal XM_Position of ID %s = %s" % (XM_DXL_ID[dxl_id], xm_position[dxl_id]))
                self.packet_handler_xm.write4ByteTxRx(self.port_handler_xm, dxl_id, XM_ADDR_GOAL_POSITION, position_dynamixel[dxl_id][n])
            rospy.loginfo("모터 제어 value : %d, %d, %d, %d, %d", *position_dynamixel.T[n])
        self.state_done()
        print("state_done")
       
    def gripper(self, data): # gripper 제어, 사물의 물체를 받고, 그에 따른 그리퍼 모터 value 도출 및 제어, 완료 시 state_done 토픽 발행
        # Assuming we get the desired angle in degrees from the message
        goal_grip_seperation_mm = data.data ## mm
        goal_grip_seperation = 3100 - (goal_grip_seperation_mm-self.gripper_close_mm)*self.seperation_per_mm
        
        # linear traj
        goal_grip_seperation = int(goal_grip_seperation)
        grip_value_arr = np.linspace(self.current_grip_seperation, goal_grip_seperation, N_grip)
        grip_value_arr = grip_value_arr.astype(int)

        # print(goal_grip_value)        
        # print(grip_value_arr)
        #self.packet_handler_xm.write4ByteTxRx(self.port_handler_xm, gripper_DXL_ID, XM_ADDR_GOAL_POSITION, goal_grip_value)
        for n in range(N_grip):
            self.packet_handler_xm.write4ByteTxRx(self.port_handler_xm, gripper_DXL_ID, XM_ADDR_GOAL_POSITION, grip_value_arr[n])        
        
        self.current_grip_seperation = goal_grip_seperation
        print('state_done')
        self.state_done()                          

    def state_done(self): # link, gripper 제어 완료 시 state_done 토픽 발행해주는 메서드
        state_msg = Bool()
        state_msg.data = True
        self.state_finish.publish(state_msg)

    def read_motor_torque_xm(self, dxl_id): # 모터 전류값 읽는 메서드, 충돌 피드백을 위한 것이므로 추후 수정 예정
        # 모터의 현재 전류 값 읽기
        dxl_present_current, dxl_comm_result, dxl_error = self.packet_handler_xm.read2ByteTxRx(self.port_handler_xm, dxl_id, XM_CURRENT_ADDR)
        
        if dxl_comm_result != COMM_SUCCESS:
            rospy.logerr("Failed to read current for XM Motor ID: {}".format(dxl_id))
            return None
        
        # 전류 값을 부호 있는 정수로 변환
        if dxl_present_current >= 32768:  # 음수 값 처리를 위한 부호 확장
            dxl_present_current -= 65536
        
        # 방향 해석 (부호로 방향을 나타냄)
        direction = "CW" if dxl_present_current >= 0 else "CCW"
        
        return dxl_present_current, direction
        
    def monitor_torque(self): # 모터 전류값 모니터링하는 메서드, 충돌 피드백을 위한 것이므로 추후 수정 예정
        data_t = np.zeros(4)
        result = 0
        for dxl_id in XM_DXL_ID:
            torque_value = self.read_motor_torque_xm(dxl_id)
            if torque_value is not None:
                torque, direction = torque_value
                # rospy.loginfo("XM Motor ID: {}, Current Torque: {:.2f}, Direction: {}".format(dxl_id, torque, direction))  
                data_t[dxl_id] = torque
        
        self.plot_torque(data_t)
        self.impact_check.diff(data_t)
        result = self.impact_check.impact_check()
        return result
        # self.torque_array(data)

    def plot_torque(self, data_t): # 모터 전류값 Plot 해주는 메서드, 충돌 피드백을 위한 것이므로 추후 수정 예정
        msg_1 = Float32(data=data_t[0])
        msg_2 = Float32(data=data_t[1])
        msg_3 = Float32(data=data_t[2])
        msg_4 = Float32(data=data_t[3])
        msg_5 = Float32(data=data_t[4])
        
        self.pub_data_1.publish(msg_1)
        self.pub_data_2.publish(msg_2)
        self.pub_data_3.publish(msg_3)
        self.pub_data_4.publish(msg_4)
        self.pub_data_4.publish(msg_5)

    def shutdown(self): # 종료 시 포트 닫기
        # 노드 종료 시 AX, XM 시리얼 포트 닫기
        self.port_handler_xm.closePort()    
        rospy.loginfo("Shutdown Dynamixel node.")
        
    def pub_pose(self, pose):

        position_dynamixel = np.array(pose)

        position_dynamixel[0] = 1007 # base 수리 되면 변경 -> (position_dynamixel[0]+180)*self.change_para
        position_dynamixel[1] = (position_dynamixel[1]+90)*self.change_para
        position_dynamixel[2] = (position_dynamixel[2]+180+20)*self.change_para #모터 쳐짐 고려 +20
        position_dynamixel[3] = (position_dynamixel[3]+180)*self.change_para
        position_dynamixel[4] = (position_dynamixel[4]+180)*self.change_para        
        position_dynamixel = position_dynamixel.astype(int)

        for dxl_id in XM_DXL_ID:
            # print("Set Goal XM_Position of ID %s = %s" % (XM_DXL_ID[dxl_id], xm_position[dxl_id]))
            self.packet_handler_xm.write4ByteTxRx(self.port_handler_xm, dxl_id, XM_ADDR_GOAL_POSITION, position_dynamixel[dxl_id])
        rospy.loginfo("모터 제어 value : %d, %d, %d, %d, %d", *position_dynamixel)


class Pose:
    def __init__(self):
        self.goal_sub = rospy.Subscriber('goal_pose', fl, self.callback_goal)  # from IK solver
        self.grip_sub = rospy.Subscriber('grip_seperation', Float32, self.callback_grip) # from master node

        # init_setting
        self.gripper_open = 1800
        self.gripper_close = 3100

        self.gripper_open_mm = 72 #54
        self.gripper_close_mm = 15.9 #0
        self.current_grip_seperation = self.gripper_open 
        self.seperation_per_mm = (self.gripper_close/(self.gripper_open_mm-self.gripper_close_mm))

        self.goal_pose = None
        self.grip_seperation = None
        self.current_pose = None
        self.last_pose = [250, 0, 10, 30, 30] #초기값
        self.trajectory = []

    def callback_goal(self, msg):
        self.goal_pose = np.array(msg.msg)
        # 원래 알고리즘 상 현재 pose 를 기준으로 계산을 하는 거라 current pose로 넣어두긴 했는데
        # trajectory에 goal이 밀려 있는 상태에서 계산하면 문제가 발생할 수 있을 거 같습니다
        # self.current_pose -> self.last_pose 로 바꿔서도 한 번 해보세용
        self.trajectory = np.append(self.trajectory, self.cubic_trajectory(self.current_pose, self.goal_pose))

    def callback_grip(self, msg):
        self.grip_seperation = msg.msg
        goal_grip_seperation = 3100 - (self.grip_seperation - self.gripper_close_mm) * self.seperation_per_mm
        
        # linear traj
        goal_grip_seperation = int(goal_grip_seperation)
        grip_value_arr = np.linspace(self.current_grip_seperation, goal_grip_seperation, N_grip)
        grip_value_arr = grip_value_arr.astype(int)

        grip_trajectory = np.zeros((len(grip_trajectory, len(self.last_pose))))
        _pose = self.current_pose # 여기도 self.current_pose -> self.last_pose 로 바꿔서도 한 번 해보세용
        for idx, val in enumerate(grip_value_arr):
            _pose[-1] = val
            grip_trajectory[idx] = _pose

        self.trajectory = np.append(self.trajectory, grip_trajectory)

    def cubic_trajectory(self, th_i, th_f): # input : 시작각도, 나중각도, 분할 넘버 -> output : (n, N) 배열

        # N 계산 
        delta_th = np.max(np.abs(th_f - th_i)) # 모터들의 변화량 계산 후, 모든 모터에서 발생할 수 있는 최대 변화량
        min_n, max_n = 10, 50                  # 최소 및 최대 보간법 개수 설정
        max_delta_th = 100                     # 모터에서 발생할 수 있는 최대 변화량
        N = min_n + (delta_th / max_delta_th) * (max_n - min_n)


        t = np.linspace(0, 1, N)
        
        # 3차 다항식 계수: 초기 속도와 최종 속도를 0으로 설정 (s_curve)
        a0 = th_i[:, np.newaxis]
        a1 = 0
        a2 = 3 * (th_f - th_i)[:, np.newaxis]
        a3 = -2 * (th_f - th_i)[:, np.newaxis]
        
        # 3차 다항식을 통해 각도를 계산 (각 행이 하나의 trajectory)
        theta = a0 + a1 * t + a2 * t**2 + a3 * t**3  
        theta = theta.astype(int).reshape(N, -1)
        theta = theta.T # 데이터 타입 맞춰주기 위한 전치

        return theta
    
    def pose_update(self, current_pose):
        self.current_pose = current_pose
        if len(self.trajectory) > 0:
            self.last_pose = self.trajectory[0]
            self.trajectory.pop(0)
    
        
def main():
    rospy.init_node('motor_control', anonymous=True)
    rate = rospy.rate(30)

    pose = Pose()
    impact = Impact()
    dynamixel = DynamixelNode()

    while not rospy.is_shutdown():
        dynamixel.pub_pose(pose.last_pose)
        pose.pose_update(dynamixel.present_position)

        rate.sleep()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass