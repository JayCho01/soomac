import rospy
import numpy as np
import math
from std_msgs.msg import Float32MultiArray as fl
from std_msgs.msg import Bool, Float32
from std_msgs.msg import String
from ikpy.chain import Chain
from ikpy.link import OriginLink, URDFLink

#####################################################################################################################################################################################
# Robot configuration parameter

def dtr(dgree):
   return dgree*(np.pi/180)

l = [50, 65.95, 332.5, 270.2, 81.75, 17.25+132.47]

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
        
        self.angles = np.r_[self.angles, 0]
        print(self.angles)
        return self.angles
   
#####################################################################################################################################################################################   
# FSM
class FSM:
    def __init__(self):
        # 초기값 세팅
        self.state_done = False
        self.grip_open = 72
        self.grip_seperation = self.grip_open
        self.state = "parking"
        self.last_state = "parking"
        self.arm = MakeChain()

        # 고정된 위치
        self.parking = np.array([-90, 100, -125, -70, 0]) # parking 자세 설계팀과 상의 필요 # 각도값 조절 필요, 일단 카메라 포즈랑 동일하게 해둠
        self.init_pose = np.array([-90, 100, -125, -70, 0]) # 초기 자세 # GUI에서 실행 버튼 및 초기 위치 버튼 누르면 여기로 이동함
        # self.camera_pose = np.array([-90, 100, -125, -70]) # camera_pose -> init_pos가 camera_pos를 겸하도록 변경 
        
        # offset 관련 parameter
        self.above_offset = np.array([0, 0, 120, 0])
        self.grip_offset = np.array([0, 0, 30, 0])
        self.lift_offset = np.array([0, 0, 150, 0])
        self.object_size = None  # 초기값 설정

        # 변수 사전 선언 // [0,0,0]으로 좌표 설정 시 해당 위치로 이동하라는 신호가 오면 base로 endpoint가 위치하려 하는 상황이 생기므로 초기값은 안전하게 설정함.
        self.pick_pose = [150, 0, 100, 0]
        self.place_pose = [150, 0, 100, 0]
        self.action_setting()

        ############################################################
        # action 정의
        self.action_list =["parking", "init_pose", # 고정된 위치 동작
                           "pick_above", "pick_grip", "grip_on", "pick_lift", # pick zone 동작 
                           "place_above", "place_grip", "grip_off", "place_lift"] # place zone 동작
        
        self.action_num = len(self.action_list)

        ###########################################################
        # ROS publish
        self.pub_goal_pose = rospy.Publisher('goal_pose', fl, queue_size=10) # 목표 pose에 대한 각도를 보낸다((,5)).
        self.pub_grip_seperation = rospy.Publisher('grip_seperation', Float32, queue_size=10) # fl -> Float32로 변경, 실수값 1개는 array로 못넘김

    def move_to_init(self): # init pose로 direct 이동, GUI에서 초기 위치 누르면 해당 메서드 동작
        self.pub_pose(option=1) # option이라는 단어를 반드시 명시할 것.
        print('move_to_init pub')
        self.state = "init_pose"

    def action_setting(self, print_op=False): # vision으로 부터 받은 정보를 바탕으로 way point의 좌표 정보 제작
        # pick zone
        self.pick_above = self.pick_pose + self.above_offset 
        self.pick_grip = self.pick_pose + self.grip_offset
        self.pick_lift = self.pick_pose + self.lift_offset

        # place zone
        self.place_above = self.place_pose + self.above_offset
        self.place_grip = self.place_pose + self.grip_offset
        self.place_lift = self.place_pose + self.lift_offset
        if print_op == True:
            print('\nPick : ', self.pick_above, ' -> ', self.pick_grip, ' -> ', self.pick_lift)
            print('\nPlace : ', self.place_above, ' -> ', self.place_grip, ' -> ', self.place_lift)

    def rotate_x(self, vector, degree):
        # Degree to Radian
        rad = np.radians(degree)

        # Rotation Matrix
        rotation_matrix = np.array([
            [1, 0, 0],
            [0, np.cos(rad), -np.sin(rad)],
            [0, np.sin(rad), np.cos(rad)]
        ])

        return np.dot(rotation_matrix, vector)


    def translate(self, vector, translation):
        return vector + translation      


    def transformation(self, pose):

        self.translate_vector = np.array([0, 0.2, 0.3])
        self.rotate_degree = -30
       
        vector = pose[:3]

        rotated_vector = self.rotate_x(vector, self.rotate_degree)

        translated_vector = self.translate(rotated_vector, self.translate_vector)

        print(translated_vector)

        return np.r_[translated_vector, pose[3]]

    # 비전으로터 받는 데이터 형식 {"pick": (x, y, z, theta, grip size), "place": (x, y, z, heading)}
    def get_data_from_vision(self,data): # vision으로 부터 받은 데이터를 가공해줌 -> 이후 action_setting 진행 -> 이후 new_state로 넘어가서 새로운 동작 진행
        vision_data = data
        self.object_size = np.array(vision_data[4])

        self.pick_pose = np.array(list(vision_data[:4]))
        self.place_pose = np.array(list(vision_data[5:]))
        self.pick_pose = self.transformation(self.pick_pose)
        self.place_pose = self.transformation(self.place_pose)

        self.action_setting(print_op=True) # vision data 기반으로 way point 설정, # print option은 vision data로 가공된 정보 확인하기 위함
        self.new_state() # init에서 vision 정보 받았으니 새로운 state로 이동 시 물체 위로 이동

    def update(self): #new_state로 부터 갱신된 동작을 진행해줌.
        # 초기화
        if self.state == "init_pose":            
            self.pub_pose(option=1)
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
            self.pub_grip(self.grip_open)
            print("updated to grip_off")

        elif self.state == "place_lift":
            self.pub_pose(self.place_lift)
            print("updated to place_lift")

    ######################################################################################################

    def new_state(self): # state_done 시 방금 도착한 자세가 init이 아닌 경우 새로운 state로 변경해줌. 이후에는 바로 동작 update 진행
        # state 변수를 다음 state로 변경
        current_state_index = self.action_list.index(self.state)
        print('current state:', self.state, '(index:', current_state_index,')')

        if current_state_index == self.action_num-1: # place_offset(마지막)인 경우 init_pos로 이동 // init_pos로 이동 시 state_done topic이 와도 다음 동작으로 넘어가지 않음. 다시 vision topic 올때 까지 기다림
            print("Pick and place 완료")
            self.last_state = self.state
            self.state = self.action_list[1] # init_pose로 이동
            print('-- next state:', self.state)
            self.update()

        else:
            self.last_state = self.state
            self.state = self.action_list[current_state_index+1]
            print('-- next state:', self.state)
            self.update()

    def pub_pose(self, goal_pose=[150, 0, 100, 0], option=0): # publish 좌표 + 손목 각도[,4] -> 5축 각도[,5] # goal_pose가 없는 init, parking일 경우에는 goal_pose는 임의의 값으로 설정(어차피 안씀)

        if option == 0 : #IK 계산
            self.goal_degree = self.arm.IK(goal_pose)
            print('---- goal pose(%.2f %.2f %.2f)' % (goal_pose[0], goal_pose[1], goal_pose[2]))
        
        elif option == 1 : #사진 촬영 Pose(init_pose)
            self.goal_degree = self.init_pose

        elif option == 2 : #거치대 Pose
            self.goal_degree = self.parking

        goal_msg = fl() 
        goal_msg.data = self.goal_degree
        self.pub_goal_pose.publish(goal_msg)
        print('---- goal axis(%.2f %.2f %.2f %.2f) ori(%.2f deg)' % (self.goal_degree[0], self.goal_degree[1], self.goal_degree[2], self.goal_degree[3], self.goal_degree[4]))

    def pub_grip(self, grip_seperation): # gripper로 집어야할 사물의 길이 보내줌
        grip_msg = Float32()
        grip_msg.data = grip_seperation
        self.pub_grip_seperation.publish(grip_msg)
        print('grip seperation:', grip_seperation)

    def impact_feedback(self): # 미완성, 추후 추가 예정
        rospy.loginfo('subscribed - impact detected! going back to last state')
        self._state = self._last_state
        self.update(self._current_obj_pose)
        rospy.loginfo('freeze for 10s')

class Callback:
    def __init__(self):
        self.soomac_fsm = FSM()
        # self.soomac_fsm.move_to_init() #이거 풀면 master node 실행 시 자동으로 init_pos로 이동
        self.ros_sub()

    def ros_sub(self): # main_loop
        rospy.Subscriber('vision', fl, self.vision)         
        rospy.Subscriber('task_type', String, self.task_type)
        rospy.Subscriber('state_done', Bool, self.state_done)
        rospy.Subscriber('impact_feedback', Bool, self.impact) # 구현 예정
        rospy.spin()

    def vision(self, data): # camera -> 동작
        rospy.loginfo('vision topic is subed')
        rospy.loginfo('subscribed - object initial pos(%.2f %.2f %.2f) ori(%.2f deg), object size(%.2f), goal pos(%.2f %.2f %.2f) ori(%.2f deg)', *data.data)
        self.soomac_fsm.get_data_from_vision(data.data)

    def task_type(self, data): # gui로 부터 task받을 시 동작 메서드

        if data.data == "gui_start" : # 주차(or 어디든) -> init # 초가 주차 상태일 때 누르면 init으로 옴
            rospy.loginfo('gui_start topic is subed')
            self.soomac_fsm.move_to_init()

        elif data.data == "gui_stop" :
            rospy.loginfo('gui_stop topic is subed')

        elif data.data == "gui_pause" :
            rospy.loginfo('gui_pause topic is subed')

        elif data.data == "gui_init_pose" :
            self.soomac_fsm.move_to_init()
            rospy.loginfo('gui_init_pose topic is subed')
            
    def state_done(self, data): # motor_control로 부터 state_done 받을 시 동작 메섣,
        print('state_done\n')
        if self.soomac_fsm.state != "init_pose": # init_pose로 이동 완료된 상황에서는 new_state안하고 vision 정보 기다림
            self.soomac_fsm.new_state()
        
    def impact(self, data): # motor_control로 부터 impact 감지시 동작 메서드, 추후 제작 예정
        rospy.loginfo('impact topic is subed')

#####################################################################################################################################################################################
# main
def main():
    rospy.init_node('soomac_master_node', anonymous=True)
    Callback()

if __name__ == '__main__':
    try:
        rospy.logwarn("Master Node is on")
        main()

    except rospy.ROSInterruptException:
        rospy.loginfo('Program is shut down')
