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
from std_msgs.msg import Float32, Bool, String
# import threading
# import matplotlib.pyplot as plt
# import matplotlib.animation as animation
# import plotly.graph_objects as go
DEVICENAME = '/dev/ttyUSB0'


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
XM_BAUDRATE = 1000000

XM_TORQUE_ENABLE = 1
XM_TORQUE_DISABLE = 0

#xm_packet_handler = PacketHandler(XM_PROTOCOL_VERSION)

# 각 모터에 대한 ID 및 레지스터 주소 설정
#XM_DXL_ADDR_PRESENT_POSITION = 126  # XM 모터의 현재 위치 값을 읽기 위한 레지스터 주소


################################################################################################################################################
# parameter
N = 50 # link motor 경로 분활
N_grip = 20 # gripper motor 경로 분활
# timer = time.time()
# repeat_time = 0.05
# impact_num = 13

def degree_to_dynamixel_value(degree): # input : (1,6) -> output : (1,6) // 5축 degree + gripper_value -> 6개의 모터 value
    
    change_para = 1024/90
    position_dynamixel = np.array(degree[:5])

    position_dynamixel[0] = (position_dynamixel[0]+180)*change_para
    position_dynamixel[1] = (position_dynamixel[1]+90)*change_para
    position_dynamixel[2] = (position_dynamixel[2]+180)*change_para
    position_dynamixel[3] = (position_dynamixel[3]+180)*change_para
    position_dynamixel[4] = (position_dynamixel[4]+180)*change_para        
    position_dynamixel = position_dynamixel.astype(int)
    position_dynamixel = np.append(position_dynamixel, int(degree[5]))
    return position_dynamixel

def cubic_trajectory(th_i, th_f): # input(degree) : 시작각도, 나중각도 -> output : (n, N) 배열

    # N 계산 
    delta_th = np.max(np.abs(th_f - th_i)) # 모터들의 변화량 계산 후, 모든 모터에서 발생할 수 있는 최대 변화량
    min_n, max_n = 10, 40                 # 최소 및 최대 보간법 개수 설정
    max_delta_th = 100                     # 모터에서 발생할 수 있는 최대 변화량
    global N 
    N = min_n + (delta_th / max_delta_th) * (max_n - min_n)    
    N = int(N)
    if N > max_n:
        N = max_n
    t = np.linspace(0, 1, N)
    
    # 3차 다항식 계수: 초기 속도와 최종 속도를 0으로 설정 (s_curve)
    a0 = th_i[:, np.newaxis]
    a1 = 0
    a2 = 3 * (th_f - th_i)[:, np.newaxis]
    a3 = -2 * (th_f - th_i)[:, np.newaxis]
    
    # 3차 다항식을 통해 각도를 계산 (각 행이 하나의 trajectory)
    theta = a0 + a1 * t + a2 * t**2 + a3 * t**3  
    print('trajectory N : ',N)
    return theta.T

class Impact: # 작업 완료
    def __init__(self):
        self.last_torgue = np.array([])
        self.diff_torques = np.array([]).reshape(0,5)
        self.diff_1st = rospy.Publisher('/diff_1st', Float32, queue_size=10)
        self.diff_2nd = rospy.Publisher('/diff_2nd', Float32, queue_size=10)
        self.impact_to_gui = rospy.Publisher('/impact_to_gui', Bool, queue_size=10)

    def diff(self, current_torque): # 입력 : 현재 토크 값, 출력은 없으며 전역 변수 diff_torques에 토크 변화량이 저장됨
        # print("#######미분#######")
        current_torque = np.array(current_torque)
        diff_torque = np.zeros(5)
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
        self.diff_1st.publish(diff_torque[1])

    def impact_check(self):
        #print("#######충격 확인#######")
        diff_torques_num = self.diff_torques.shape[0] # 저장된 변화량의 개수
        # dof = self.diff_torques.shape[1] # 모터의 개수(5)
        diff_2nd = np.zeros(5) # 토크 변화량의 기울기가 급변할 때를 파악하기 위함.

        result = 0 # 충돌 결과
        if self.diff_torques.shape[0] >= 2: # 1차 미분값이 2개 이상 모이면
            for i in range(5):
                diff_2nd[i] = self.diff_torques[diff_torques_num-1][i]-self.diff_torques[diff_torques_num-2][i]
            diff_2nd = np.abs(diff_2nd)
            
            if diff_2nd[0] >= 70: # 수평 방향 충돌 감지
                result = 1
                rospy.logerr('########## impact 발생(수평) ')

            if diff_2nd[1] >= 100: # XM_1 방향 충돌 감지
                result = 1
                rospy.logerr('########## impact 발생(XM_1) ')

            if diff_2nd[2] >= 70: # 수직 방향 충돌 감지
                result = 1
                rospy.logerr('########## impact 발생(수직)')

            if diff_2nd[0]/70 + diff_2nd[2]/70 >= 1.5: # 수직 방향 충돌 감지
                result = 1
                rospy.logerr('########## impact 발생(대각선)')
            
        self.diff_2nd.publish(diff_2nd[1])                

        return result
            
    def print_info(self, test_data):
        print("input data : ",test_data)
        print("last_torque : ",self.last_torgue)
        print("diff_torques : ",self.diff_torques)
        print("last_diff_2nd : ",self.last_diff_2nd)


class DynamixelNode:
    def __init__(self): # 파라미터 설정
        # setting
        self.impact = Impact()
        self.dynamixel_setting()
        self.change_para = 1024/90

        # init_setting
        self.gripper_open = 3300 # 3500
        self.gripper_close = 2924 #2236

        self.gripper_open_mm = 45 #49 -> 
        self.gripper_close_mm = 0 #15.9
        self.current_grip_seperation = self.gripper_open
        self.seperation_per_mm = (self.gripper_close/(self.gripper_open_mm-self.gripper_close_mm))
        
        self.packet_handler_xm.write4ByteTxRx(self.port_handler_xm, gripper_DXL_ID, XM_ADDR_GOAL_POSITION, self.current_grip_seperation) #실행 시 그리퍼 open

        ## ROS
        # for state done


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
        
        self.packet_handler_xm.write4ByteTxRx(self.port_handler_xm, 0, XM_ADDR_POSITION_P_GAIN, 100) 
        self.packet_handler_xm.write4ByteTxRx(self.port_handler_xm, 1, XM_ADDR_POSITION_P_GAIN, 200)
        self.packet_handler_xm.write4ByteTxRx(self.port_handler_xm, 2, XM_ADDR_POSITION_P_GAIN, 100) 
        self.packet_handler_xm.write4ByteTxRx(self.port_handler_xm, 3, XM_ADDR_POSITION_P_GAIN, 200) 
        self.packet_handler_xm.write4ByteTxRx(self.port_handler_xm, 4, XM_ADDR_POSITION_P_GAIN, 400) 

        
        # p gain 설정
        for i in range(0,5):
            #self.packet_handler_xm.write4ByteTxRx(self.port_handler_xm, i, XM_ADDR_POSITION_P_GAIN, 400) #800 -> 200``
            #self.packet_handler_xm.write4ByteTxRx(self.port_handler_xm, i, XM_ADDR_POSITION_I_GAIN, 5) #0
            self.packet_handler_xm.write4ByteTxRx(self.port_handler_xm, i, XM_ADDR_POTISION_D_GAIN, 10) #0
            #self.packet_handler_xm.write4ByteTxRx(self.port_handler_xm, i, XM_ADDR_VELOCITY_I_GAIN, 1920) #1920
            #self.packet_handler_xm.write4ByteTxRx(self.port_handler_xm, i, XM_ADDR_VELOCITY_P_GAIN, 200) #100
            self.packet_handler_xm.write4ByteTxRx(self.port_handler_xm, i, XM_ADDR_FEEDFORWARD_1ST_GAIN, 10) #0
            #self.packet_handler_xm.write4ByteTxRx(self.port_handler_xm, i, XM_ADDR_FEEDFORWARD_2ND_GAIN, 20) #0

    def read_motor_position(self, port_handler, packet_handler, dxl_id, addr_present_position): # 현재 모터 value 도출해주는 메서드
        # 모터의 현재 위치 읽기
        dxl_present_position, _, _ = packet_handler.read2ByteTxRx(port_handler, dxl_id, addr_present_position)
        return dxl_present_position

    def move_current_to_goal(self, goal_pose): # 현재 각도 읽고, 목표 각도 까지 trajectory 만들어서 제어, input : 목표 각도(출발 각도는 받지 않아도 모터 자체에서 현재 각도 확인 후 traj)

        goal_dynamixel = degree_to_dynamixel_value(goal_pose)
        present_position = np.zeros(6)
        print('goal degree : ', goal_pose[:5], 'grip : ', goal_pose[5])

        for dxl_id in range(0,6):
            present_position[dxl_id] = self.read_motor_position(self.port_handler_xm, self.packet_handler_xm, dxl_id, XM_ADDR_PRESENT_POSITION)

        print('현재 : ', present_position)
        print('목표 : ', goal_dynamixel)
        traj_arr = cubic_trajectory(present_position, goal_dynamixel) # 각 모터마다의 각도를 trajectory, traj_arr는 (5,N)의 shape
        
        position_dynamixel = traj_arr        
        position_dynamixel = position_dynamixel.astype(int)

        for n in range(N):
            for dxl_id in range(0,6):
                # print("Set Goal XM_Position of ID %s = %s" % (XM_DXL_ID[dxl_id], xm_position[dxl_id]))
                self.packet_handler_xm.write4ByteTxRx(self.port_handler_xm, dxl_id, XM_ADDR_GOAL_POSITION, position_dynamixel[n][dxl_id])
            
            rospy.loginfo("모터 제어 value : %d, %d, %d, %d, %d, %d", *position_dynamixel[n])
        
        print("move_currnet_to_goal is done")

    def pub_pose(self, pose):
        dynamixel_value = degree_to_dynamixel_value(pose) 
        # print(dynamixel_value)

        for dxl_id in XM_DXL_ID:
            # print("Set Goal XM_Position of ID %s = %s" % (XM_DXL_ID[dxl_id], xm_position[dxl_id]))
            self.packet_handler_xm.write4ByteTxRx(self.port_handler_xm, dxl_id, XM_ADDR_GOAL_POSITION, dynamixel_value[dxl_id])
        
        self.packet_handler_xm.write4ByteTxRx(self.port_handler_xm, gripper_DXL_ID, XM_ADDR_GOAL_POSITION, dynamixel_value[5]) ## 그리퍼 별도 제어
        # rospy.loginfo("모터 제어 degree : %.2f, %.2f, %.2f, %.2f, %.2f // gripper : %.2f", *pose)
        # rospy.loginfo("모터 제어 value : %d, %d, %d, %d, %d // gripper : %.2f", *dynamixel_value)       

    def shutdown(self): # 종료 시 포트 닫기
        # 노드 종료 시 AX, XM 시리얼 포트 닫기
        self.port_handler_xm.closePort()    
        rospy.loginfo("Shutdown Dynamixel node.")
        
    def read_motor_current(self, dxl_id): # 주어진 id의 모터에 대한 전류 값 반환
        # 모터의 현재 전류 값 읽기
        dxl_present_current, dxl_comm_result, dxl_error = self.packet_handler_xm.read2ByteTxRx(self.port_handler_xm, dxl_id, XM_CURRENT_ADDR)
        
        if dxl_comm_result != COMM_SUCCESS:
            # rospy.logerr("Failed to read current for XM Motor ID: {}".format(dxl_id))
            return None
        
        # 전류 값을 부호 있는 정수로 변환
        if dxl_present_current >= 32768:  # 음수 값 처리를 위한 부호 확장
            dxl_present_current -= 65536
        
        return dxl_present_current
        
    def monitor_current(self): # 모터 전류값 모니터링하는 메서드. 모니터링 후 충격 확인까지 진행
        data_t = np.zeros(5)
        result = 0
        for dxl_id in XM_DXL_ID:
            torque_value = self.read_motor_current(dxl_id)
            if torque_value is not None:
                # rospy.loginfo("XM Motor ID: {}, Current Torque: {:.2f}, Direction: {}".format(dxl_id, torque, direction))  
                data_t[dxl_id] = torque_value
        
        self.plot_torque(data_t)
        self.impact.diff(data_t)
        result = self.impact.impact_check()
        return result
        
    def plot_torque(self, data_t): # 모터 전류값 Plot 해주는 메서드
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


class Pose:
    def __init__(self):
        self.goal_sub = rospy.Subscriber('/goal_pose', fl, self.callback_goal)  # from IK solver
        self.grip_sub = rospy.Subscriber('/grip_seperation', Float32, self.callback_grip) # from master node
        self.taks_sub = rospy.Subscriber('/task_to_motor_control', String , self.callback_task) # from master node
        self.stop_state = False
        # init_setting
        
        self.gripper_open = 3300
        self.gripper_close = 2236

        self.gripper_open_mm = 45 #72
        self.gripper_close_mm = 0 #15.9
        self.current_grip_seperation = self.gripper_open 
        self.seperation_per_mm = ((self.gripper_open-self.gripper_close)/(self.gripper_open_mm-self.gripper_close_mm))

        self.goal_pose = None
        self.grip_seperation = None
        self.current_pose = None
        self.init_pose = np.array([0, 150, -90, -110, 0])

        self.last_pose = np.append(self.init_pose, self.gripper_open) #pose 초기값(그리퍼 포함)
        self.trajectory = []
        self.state_done_topic = rospy.Publisher('/state_done', Bool, queue_size=10) # to master
        self.change_para = 1024/90
        self.state_done_okay = False


    def callback_task(self, data):
        print('\n---------------- task callback ----------------')
        if data.data == 'stop':
            self.stop_state = True

        if data.data == 'continue':
            print('continue')
            self.stop_state = False  

        if data.data == 'previous':
            print('previous')
            self.trajectory = self.trajectory[:1, :]
            self.trajectory[0] = self.last_pose
            rospy.sleep(0.1) # trjectory 최신화 될때 까지 기달
            self.stop_state = False
             
        if data.data == 'camera': # init
            print("camera")
            # self.trajectory = [] # trajectory 대기열 삭제        
            # self.trajectory = cubic_trajectory(self.last_pose[:5], self.init_pose)
            # grip = self.gripper_open
            # # self.init_pose_with_grip = np.append(self.init_pose, grip) # init_pose로 설정            
            # grip_arr = np.full((self.trajectory.shape[0], 1), self.last_pose[5])
            # self.trajectory = np.hstack((self.trajectory, grip_arr))            
            # print(self.last_pose)
            self.stop_state = False

    def callback_goal(self, msg):
        self.state_done_okay = False
        self.goal_pose = np.array(msg.data)
        # 원래 알고리즘 상 현재 pose 를 기준으로 계산을 하는 거라 current pose로 넣어두긴 했는데
        # trajectory에 goal이 밀려 있는 상태에서 계산하면 문제가 발생할 수 있을 거 같습니다
        # self.current_pose -> self.last_pose 로 바꿔서도 한 번 해보세용
        print('### callback_goal ###')
        print('현재 상태 : ', self.last_pose[:5])
        print('목표 상태 : ', self.goal_pose)
        self.trajectory = cubic_trajectory(self.last_pose[:5], self.goal_pose)
        grip_arr = np.full((self.trajectory.shape[0], 1), self.last_pose[5])
        self.trajectory = np.hstack((self.trajectory, grip_arr))
        # print(self.trajectory)
        # self.trajectory = np.append(self.trajectory, cubic_trajectory(self.last_pose, self.goal_pose))
        
    def callback_grip(self, msg):
        self.state_done_okay = False
        self.grip_seperation = msg.data
        goal_grip_seperation = self.gripper_close + (self.grip_seperation - self.gripper_close_mm) * self.seperation_per_mm

        print('### callback_grip ###')
        print('현재 상태 : ', self.last_pose[5])
        print('목표 상태 : ', goal_grip_seperation)        
        # linear traj
        goal_grip_seperation = int(goal_grip_seperation)
        grip_value_arr = np.linspace(self.current_grip_seperation, goal_grip_seperation, N_grip)
        grip_value_arr = grip_value_arr.astype(int)
        self.trajectory = np.tile(self.last_pose[:5], (N_grip, 1))
        self.trajectory = np.column_stack((self.trajectory,grip_value_arr)) 

    def state_done(self): # link, gripper 제어 완료 시 state_done 토픽 발행해주는 메서드
        if self.state_done_okay == False:
            state_msg = Bool()
            state_msg.data = True
            self.state_done_topic.publish(state_msg)
        self.state_done_okay = True


    def pose_update(self):
        if len(self.trajectory) > 1: # trajectory 대기열에 2개 이상 존재 시 가장 앞 값을 last_pose에 넣고 해당 값을 삭제.
            
            self.last_pose = self.trajectory[0]
            self.trajectory = np.delete(self.trajectory,0,0) # 대기열 첫번째 값 삭제
        
        elif len(self.trajectory) == 1: # 값이 1개라면 traj 모두 진행 후 목표 값에 도달해있다는 뜻임. 따라서 state_done 발행
            self.last_pose = self.trajectory[0] # traj의 마지막 인자로 last_pose 설정
            self.state_done() # state_done 토픽 발행 시, moster_node로 부터 goal_pose가 들어오고, trajectory 배열이 추가로 생성됨
            # print("state_done\n\n")
            
        elif len(self.trajectory) == 0: # 값이 0이라면, 아직 한번도 trajectory를 진행하지 않은 상태임. 해당 경우는 초기 실행 상태임으로 pose update 없이 pass
            pass

def main(data):
    print('motor_control_node is started')
    dynamixel = DynamixelNode()
    pose = Pose()
    impact = Impact()
    define_pose = data.data # (,5)
    define_pose = np.append(define_pose, pose.gripper_open) #pose 초기값(그리퍼 포함)
    dynamixel.move_current_to_goal(define_pose) # 초기 실행 시, 임의의 자세에서 last_pose로 이동. 이때 last_pose는 초기 pose임
    pose.state_done()

    while not rospy.is_shutdown():
        impact_state = dynamixel.monitor_current()
        if impact_state == 1:
            pose.stop_state = True 
            impact.impact_to_gui.publish(True)
            
        dynamixel.pub_pose(pose.last_pose) # 계속 last_pose로 모터 작동
        if pose.stop_state == False: # stop이 아니면 pose update
            pose.pose_update()

if __name__ == '__main__':
    try:
        rospy.init_node('motor_control', anonymous=True)
        print("waiting for start")
        start_sub = rospy.Subscriber('/start', fl, main)  # from IK solver
        rospy.spin()
    except rospy.ROSInterruptException:
        pass