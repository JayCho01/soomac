import rospy
import numpy as np
from std_msgs.msg import Float32MultiArray as fl
from std_msgs.msg import Bool

import matplotlib.pyplot as plt
from ikpy.chain import Chain
from ikpy.link import OriginLink, URDFLink

#####################################################################################################################################################################################
# method
def dtr(dgree):
   return dgree*(np.pi/180)

def quat_from_angle_axis(angle, axis):
    axis = np.array(axis)
    axis = axis / np.linalg.norm(axis)  # Normalize the axis
    half_angle = angle / 2.0
    sin_half_angle = np.sin(half_angle)
    
    w = np.cos(half_angle)
    x = axis[0] * sin_half_angle
    y = axis[1] * sin_half_angle
    z = axis[2] * sin_half_angle
    
    return np.array([w, x, y, z])

def quat_mul(q1, q2):
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    
    w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
    x = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
    y = w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2
    z = w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2
    
    return np.array([w, x, y, z])

#####################################################################################################################################################################################
# 로봇 파라미터
l = [67, 45, 338, 281, 82, 60]

d = [l[0], l[1], 0, 0, 0]
a = [0, 0, l[2], l[3], l[4]+l[5]]
al = [0, dtr(90), 0, 0, dtr(90)]
#####################################################################################################################################################################################
# ikpy
class Chain:
    def __init__(self):
        self.make_chain()

    def Make_URDF(self, link_name, d, a, al,th=0):
        return URDFLink(
            name = link_name,
            origin_translation=[a, 0, d], 
            origin_orientation=[al, 0, th],
            rotation=[0, 0, 1],
        )
  # 4-DOF robot arm define
    def make_chain(self): 
            self.arm = Chain(name='arm', links=[
            OriginLink(), # base
            self.Make_URDF('link1', d[0], a[0], al[0], th=dtr(90)),
            self.Make_URDF('link2', d[1], a[1], al[1]),
            self.Make_URDF('link3', d[2], a[2], al[2]),
            self.Make_URDF('link4', d[3], a[3], al[3]),
            self.Make_URDF('link5', d[4], a[4], al[4])],
            active_links_mask=[False, True, True, True, True, False] )

  # IK 계산 매서드 / position -> angle(4개 축)
    def IK(self, target_position):
        angle = self.arm.inverse_kinematics(target_position, target_orientation=[0, 0, -1], orientation_mode="X") #, target_orientation=[0, 0, -1], orientation_mode="X")
        # orientation mode 를 "X"로 설정하기. EE의 green axis가 x축 이므로.
        self.angles = np.round(np.rad2deg(angle), 3)
        self.angles = self.angles[1:5]
        # print(self.angles)
        return self.angles
   
#####################################################################################################################################################################################   
# FSM
class FSM:
    def __init__(self):
        # 초기값 세팅
        self.state_done = False
        self.gripper_state = False
        self.state = "parking"
        self.last_state = "parking"
        self.arm = Chain()

        # 고정된 위치
        self.parking = np.array([0, 200, 400, 0])
        self.init_pos = np.array([200, 0, 300, 0])
        
        # offset 관련 parameter
        self.above_offset = np.array([0, 0, 100, 0])
        self.grip_offset = np.array([0, 0, 30, 0])
        self.lift_offset = np.array([0, 0, 80, 0])

        # 초기 각도
        self.start_degree = self.arm.IK(self.parking[:3])
        self.start_degree = np.r_[self.start_degree, self.parking[3]]
        #print(self.start_degree)
        ############################################################
        # action 정의
        self.action_list =["parking", "init_pos", # 고정된 위치 동작
                           "pick_above", "pick_grip", "grip_on", "pick_lift", # pick zone 동작 
                           "place_above", "place_grip", "grip_off", "place_lift"] # place zone 동작
        
        self.action_num = len(self.action_list)

        ###########################################################
        self.hand_down_quat = [0.0, 0.0, -0.707107, 0.707107]
        grab_angle = np.pi / 6.0
        grab_axis = [0, 0, 1]
        grab_quat = quat_from_angle_axis(grab_angle, grab_axis)
        self.obj_grab_quat = quat_mul(grab_quat, self.hand_down_quat)
        ###########################################################
        # ROS
        self.pub_goal_pose = rospy.Publisher('goal_pose', fl, queue_size=10)
        self.pub_grip_state = rospy.Publisher('grip_state', Bool, queue_size=10)
    
    def action_setting(self): # vision으로 부터 각 동작 지점의 좌표 정보 제작
        ## 각 동작 별 위치 계산
        # pick zone
        self.pick_above = self.pick_pos + self.above_offset 
        self.pick_grip = self.pick_pos + self.grip_offset
        self.pick_lift = self.pick_pos + self.lift_offset

        # place zone
        self.place_above = self.place_pos + self.above_offset
        self.place_grip = self.place_pos + self.grip_offset
        self.place_lift = self.place_pos + self.lift_offset

    def get_data_from_vision(self,data): # vision으로 부터 받은 데이터(3개의 위치, 1개의 손목 각도) 가공
        vision_Data = data
        self.pick_pos = np.array(list(vision_Data[:4]))
        self.place_pos = np.array(list(vision_Data[4:]))
        self.action_setting()
        self.new_state()

    def update(self):
############################################################################################################################################################################
        if self.state == "init_pos":            
            self.pub_pos(self.init_pos)
            print('update done to init_pos')
            print('######################################################################################')
            
############################################################################################################################################################################
        if self.state == "pick_above":
            self.pub_pos(self.pick_above)
            print('update done to pick_above')
            print('######################################################################################')
            

        if self.state == "pick_grip":
            self.pub_pos(self.pick_grip)
            print('update done to pick_grip')
            print('######################################################################################')


        if self.state == "grip_on":
            self.gripper_state = True
            self.pub_grip(self.gripper_state)
            print("update done to grip_on")
            print('######################################################################################')


        if self.state == "pick_lift":
            self.pub_pos(self.pick_lift)
            print("update done to pick_lift")
            print('######################################################################################')
############################################################################################################################################################################
        if self.state == "place_above":
            self.pub_pos(self.place_above)
            print("update done to place_above")
            print('######################################################################################')

        if self.state == "place_grip":
            #self.pub_pos(self.place_grip)
            print("update done to place_grip")
            print('######################################################################################')

        if self.state == "grip_off":
            self.gripper_state = False
            self.pub_grip(self.gripper_state)
            print('######################################################################################')

        if self.state == "place_lift":
            self.pub_pos(self.place_lift)
            print("update done to place_lift")
            print('######################################################################################')
############################################################################################################################################################################

    def new_state(self):
        ##  state 변수를 다음 state로 변경
        current_state_index = self.action_list.index(self.state)
        print('기존 동작: ', self.state, '동작 번호', current_state_index)

        if current_state_index == self.action_num-1: # "place_offset"인 경우
            print("Pick and place 완료")
            self.last_state = self.state
            self.state = self.action_list[1] # init_pos로 이동
            print('나중 동작 : ', self.state)
            self.update()

        elif current_state_index != self.action_num:
            self.last_state = self.state
            self.state = self.action_list[current_state_index+1]
            print('나중 동작 : ', self.state)
            self.update()

    def pub_pos(self, goal_pose): #publish 좌표 + 손목 각도
        goal_msg = fl() # 통신 메세지 자료형 설정
        self.goal_degree = self.arm.IK(goal_pose[:3])

        self.goal_degree = np.r_[self.goal_degree, goal_pose[3]] # 3차원 좌표 + 손목 각도
        goal_msg.data = np.r_[self.start_degree, self.goal_degree]
        
        self.pub_goal_pose.publish(goal_msg)
        rospy.loginfo('goal : pos(%.2f %.2f %.2f) ori(%.2f deg)', *goal_pose)
        rospy.loginfo('start degree / axis(%.2f %.2f %.2f %.2f) ori(%.2f deg)', *self.start_degree)
        rospy.loginfo('goal degree / axis(%.2f %.2f %.2f %.2f) ori(%.2f deg)', *self.goal_degree)
        self.start_degree = self.goal_degree

    def pub_grip(self, grip_state): #publish gripper on/off
        grip_msg = Bool()
        grip_msg.data = grip_state
        self.pub_grip_state.publish(grip_msg)
        rospy.loginfo(grip_state)

    def impact_feedback(self): #미완성
        rospy.loginfo('subscribed - impact detected! going back to last state')
        self._state = self._last_state
        self.update(self._current_obj_pose)
        rospy.loginfo('freeze for 10s')

class callback:
    def __init__(self):
        self.soomac_fsm = FSM()
        self.ros_sub()

    def ros_sub(self): # 3개로 축소, vision, gui, state_done, Impact_feedback  # gui는 string으로
        rospy.Subscriber('vision', fl, self.vision)         

        rospy.Subscriber('gui_start', Bool, self.gui_start)
        rospy.Subscriber('gui_init_pos', Bool, self.gui_init_pos)
        rospy.Subscriber('gui_stop', Bool, self.gui_stop)
        rospy.Subscriber('gui_pause', Bool, self.gui_pause)

        rospy.Subscriber('state_done', Bool, self.state_done)
        
        rospy.Subscriber('impact_feedback', Bool, self.impact) # 구현 예정
        rospy.spin()

    def vision(self, data):
        rospy.loginfo('vision topic is subed')
        rospy.loginfo('subscribed - object initial pos(%.2f %.2f %.2f) ori(%.2f deg), goal pos(%.2f %.2f %.2f) ori(%.2f deg)', *data.data)
        self.soomac_fsm.get_data_from_vision(data.data)

    def gui_start(self, data):
        rospy.loginfo('gui_start topic is subed')
        self.soomac_fsm.new_state()

    def gui_stop(self, data):
        rospy.loginfo('gui_stop topic is subed')
        ######기능 추가######

    def gui_pause(self, data):
        rospy.loginfo('gui_pause topic is subed')
        ######기능 추가######

    def gui_init_pos(self, data):
        rospy.loginfo('gui_init_pos topic is subed')
        ######기능 추가######
        #     
    def state_done(self, data):
        rospy.loginfo('state_done')
        self.soomac_fsm.new_state()
        
    def impact(self, data):
        rospy.loginfo('impact topic is subed')
        ######기능 추가######

#####################################################################################################################################################################################
# main
def main():
    rospy.init_node('master', anonymous=True)
    callback()

#####################################################################################################################################################################################

if __name__ == '__main__':
    try:
        rospy.logwarn("Master Node is on")
        print('######################################################################################')
        main()

    except rospy.ROSInterruptException:
        rospy.loginfo('Program is shut down')