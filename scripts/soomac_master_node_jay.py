import rospy
import numpy as np
import math
from std_msgs.msg import Float32MultiArray as fl
from std_msgs.msg import Bool
from std_msgs.msg import String
from ikpy.chain import Chain
from ikpy.link import OriginLink, URDFLink

#####################################################################################################################################################################################
# Robot configuration parameter

def dtr(dgree):
   return dgree*(np.pi/180)

l = [67, 45, 338, 281, 82, 60]

d = [l[0], l[1], 0, 0, 0]
a = [0, 0, l[2], l[3], l[4]+l[5]]
al = [0, dtr(90), 0, 0, dtr(90)]

#####################################################################################################################################################################################
# Ikpy
class MakeChain:
    def __init__(self):
        self.make_chain()

    def Make_URDF(self, link_name, d, a, al, th=0):
        return URDFLink(
            name = link_name,
            origin_translation=[a, 0, d], 
            origin_orientation=[al, 0, th],
            rotation=[0, 0, 1],
        )
    
    #4-DOF robot arm define
    def make_chain(self): 
            self.arm = Chain(name='arm', links=[
            OriginLink(), # base
            self.Make_URDF('link1', d[0], a[0], al[0], th=dtr(90)),
            self.Make_URDF('link2', d[1], a[1], al[1]),
            self.Make_URDF('link3', d[2], a[2], al[2]),
            self.Make_URDF('link4', d[3], a[3], al[3]),
            self.Make_URDF('link5', d[4], a[4], al[4])],
            active_links_mask=[False, True, True, True, True, False] )

    #IK 계산 매서드 / position -> angle(4개 축)
    def IK(self, target_pose):
        angle = self.arm.inverse_kinematics(target_pose[:3], target_orientation=[0, 0, -1], orientation_mode="X") #, target_orientation=[0, 0, -1], orientation_mode="X")
        # orientation mode 를 "X"로 설정하기. EE의 green axis가 x축 이므로
        self.angles = np.round(np.rad2deg(angle), 3)
        self.angles = self.angles[1:5] #[0,n,n,n,n,0]

        wrist_angle_radians = math.atan2(target_pose[1], target_pose[0])
        wrist_angle_degrees = math.degrees(wrist_angle_radians) + target_pose[3] #atan로 구한 wrist의 각도를 빼서 0으로 맞춰주고, 목표 각도를 다시 더해주기

        self.angles = np.r_[self.angles, wrist_angle_degrees]
        print(self.angles)
        return self.angles
   
#####################################################################################################################################################################################   
# FSM
class FSM:
    def __init__(self):
        # 초기값 세팅
        self.state_done = False
        self.grip_open = 50
        self.grip_seperation = self.grip_open
        self.state = "parking"
        self.last_state = "parking"
        self.arm = MakeChain()

        # 고정된 위치
        self.parking = np.array([0, 200, 400, 0])
        self.init_pose = np.array([200, 0, 300, 0])
        self.camera_pose = np.array([200, 0, 300, 0])
        
        # offset 관련 parameter
        self.above_offset = np.array([0, 0, 100, 0])
        self.grip_offset = np.array([0, 0, 30, 0])
        self.lift_offset = np.array([0, 0, 80, 0])
        self.object_size = None  # 초기값 설정

        # 초기 각도
        self.start_degree = self.arm.IK(self.parking)
        #print(self.start_degree)

        # 초기 위치
        self.pick_pose = np.array([0, 0, 0, 0])  
        self.place_pose = np.array([0, 0, 0, 0]) 
        self.action_setting()

        ############################################################
        # action 정의
        self.action_list =["parking", "init_pose", # 고정된 위치 동작
                           "pick_above", "pick_grip", "grip_on", "pick_lift", # pick zone 동작 
                           "place_above", "place_grip", "grip_off", "place_lift"] # place zone 동작
        
        self.action_num = len(self.action_list)

        ###########################################################
        # ROS
        self.pub_goal_pose = rospy.Publisher('goal_pose', fl, queue_size=10)
        self.pub_grip_seperation = rospy.Publisher('grip_seperation', fl, queue_size=10)
    
    
    def action_setting(self): # vision으로 부터 각 동작 지점의 좌표 정보 제작
        
        # pick zone
        self.pick_above = self.pick_pose + self.above_offset 
        self.pick_grip = self.pick_pose + self.grip_offset
        self.pick_lift = self.pick_pose + self.lift_offset

        # place zone
        self.place_above = self.place_pose + self.above_offset
        self.place_grip = self.place_pose + self.grip_offset
        self.place_lift = self.place_pose + self.lift_offset


    # 비전으로터 받는 데이터 형식 {"pick": (x, y, z, theta, grip size), "place": (x, y, z, heading)}
    def get_data_from_vision(self,data): 
        vision_data = data
        self.object_size = np.array(vision_data[4])
        self.pick_pose = np.array(list(vision_data[:4]))
        self.place_pose = np.array(list(vision_data[5:]))
        self.action_setting()
        self.new_state()

    def update(self):
        
        # 초기화
        if self.state == "init_pose":            
            self.pub_pose(self.init_pose)
            print('updated to init_pose')
            
        # pick 
        elif self.state == "pick_above":
            self.pub_pose(self.pick_above)
            print('updated to pick_above')            

        elif self.state == "pick_grip":
            self.pub_pose(self.pick_grip)
            print('updated to pick_grip')

        elif self.state == "grip_on":
            self.grip_seperation = self.object_size
            self.pub_grip(self.grip_seperation)
            print("updated to grip_on")

        elif self.state == "pick_lift":
            self.pub_pose(self.pick_lift)
            print("updated to pick_lift")
            
        # place
        elif self.state == "place_above":
            self.pub_pose(self.place_above)
            print("updated to place_above")

        elif self.state == "place_grip":
            self.pub_pose(self.place_grip)
            print("updated to place_grip")

        elif self.state == "grip_off":
            self.grip_seperation = self.grip_open
            self.pub_grip(self.grip_seperation)
            print("updated to grip_off")

        elif self.state == "place_lift":
            self.pub_pose(self.place_lift)
            print("updated to place_lift")


    ######################################################################################################

    def new_state(self):
        #  state 변수를 다음 state로 변경
        current_state_index = self.action_list.index(self.state)
        print('current state:', self.state, '(index:', current_state_index,')')

        if current_state_index == self.action_num-1: # place_offset(마지막)인 경우
            print("Pick and place 완료")
            self.last_state = self.state
            self.state = self.action_list[1] # init_pose로 이동
            #print('-- next state:', self.state)
            self.update()

        else :
            self.last_state = self.state
            self.state = self.action_list[current_state_index+1]
            #print('-- next state:', self.state)
            self.update()

    def pub_pose(self, goal_pose, option = 0): #publish 좌표 + 손목 각도

        if option == 0 : #IK 계산
            self.goal_degree = self.arm.IK(goal_pose)

        elif option == 1 : #사진 촬영 Pose
            self.goal_degree = self.camera_pose

        elif option == 2 : #거치대 Pose
            self.goal_degree = self.parking

        goal_msg = fl() 
        goal_msg.data = np.r_[self.start_degree, self.goal_degree]
        self.pub_goal_pose.publish(goal_msg)
        print('---- goal pose(%.2f %.2f %.2f)' % (goal_pose[0], goal_pose[1], goal_pose[2]))
        print('---- start axis(%.2f %.2f %.2f %.2f) ori(%.2f deg)' % (self.start_degree[0], self.start_degree[1], self.start_degree[2], self.start_degree[3], self.start_degree[4]))
        print('---- goal  axis(%.2f %.2f %.2f %.2f) ori(%.2f deg)' % (self.goal_degree[0], self.goal_degree[1], self.goal_degree[2], self.goal_degree[3], self.goal_degree[4]))

        self.start_degree = self.goal_degree
            

    def pub_grip(self, grip_seperation):
        grip_msg = fl()
        grip_msg.data = grip_seperation
        self.pub_grip_seperation.publish(grip_msg)
        print('grip seperation:',grip_seperation)

    def impact_feedback(self): #미완성
        rospy.loginfo('subscribed - impact detected! going back to last state')
        self._state = self._last_state
        self.update(self._current_obj_pose)
        rospy.loginfo('freeze for 10s')

class Callback:
    def __init__(self):
        self.soomac_fsm = FSM()
        self.ros_sub()

    def ros_sub(self):
        rospy.Subscriber('vision', fl, self.vision)         
        rospy.Subscriber('task_type', String, self.task_type)
        rospy.Subscriber('state_done', Bool, self.state_done)
        rospy.Subscriber('impact_feedback', Bool, self.impact) # 구현 예정
        rospy.spin()

    def vision(self, data):
        rospy.loginfo('vision topic is subed')
        rospy.loginfo('subscribed - object initial pos(%.2f %.2f %.2f) ori(%.2f deg), object size(%.2f), goal pos(%.2f %.2f %.2f) ori(%.2f deg)', *data.data)
        self.soomac_fsm.get_data_from_vision(data.data)

    def task_type(self, data):

        if data.data == "gui_start" :
            rospy.loginfo('gui_start topic is subed')
            self.soomac_fsm.new_state()

        elif data.data == "gui_stop" :
            rospy.loginfo('gui_stop topic is subed')

        elif data.data == "gui_pause" :
            rospy.loginfo('gui_pause topic is subed')

        elif data.data == "gui_init_pose" :
            rospy.loginfo('gui_init_pose topic is subed')
    
            
    def state_done(self, data):
        print('state_done\n')
        self.soomac_fsm.new_state()
        
    def impact(self, data):
        rospy.loginfo('impact topic is subed')

#####################################################################################################################################################################################
# main
def main():
    rospy.init_node('master', anonymous=True)
    Callback()

if __name__ == '__main__':
    try:
        rospy.logwarn("Master Node is on")
        main()

    except rospy.ROSInterruptException:
        rospy.loginfo('Program is shut down')