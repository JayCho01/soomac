#!/usr/bin/env python

import rospy
import numpy as np
from std_msgs.msg import Float32MultiArray as fl
from std_msgs.msg import Bool

import matplotlib.pyplot as plt
from ikpy.chain import Chain
from ikpy.link import OriginLink, URDFLink

def dtr(dgree):
   return dgree*(np.pi/180)

class chain:
  def __init__(self):
    self.make_chain()
    self.angle_i = None
    self.angle_f = None

  def make_chain(self):
    # 4-DOF robot arm define
    self.arm = Chain(name='arm', links=[
        OriginLink(), # base
        URDFLink(
          name="link_1", # XM430
          origin_translation=[0, 0, 90], 
          origin_orientation=[0, 0, 0],
          rotation=[0, 0, 1],
        ),
        URDFLink( # XM430
          name="link_2",
          origin_translation=[0, 0, 35.25], 
          origin_orientation=[dtr(-90), 0, 0],
          rotation=[0, 0, 1],
        ),
        URDFLink( # AX-18(1)
          name="link_3",
          origin_translation=[158, 0, 0], # 
          origin_orientation=[0, 0, 0],
          rotation=[0, 0, 1],
        ),
        URDFLink( # AX-18(2)
          name="link_4",
          origin_translation=[173, 0, 0], # 
          origin_orientation=[0, 0, 0],
          rotation=[0, 0, 1], # not angle, just 0 or 1 / 1 means the axis of rotating
        ),
        URDFLink( # end point
          name="link_5",
          origin_translation=[73 + 51.25 + 50, 0, 0], # +하면 위로, -하면 아래로
          origin_orientation=[dtr(-90), 0, 0],
          rotation=[0, 0, 0], # no rotation(no dof)
        )        
    ], active_links_mask=[False, True, True, True, True, False]
    )

  def IK(self, target_position):
    angle = self.arm.inverse_kinematics(target_position, target_orientation=[0, 0, -1], orientation_mode="X") # orientation mode 를 "X"로 설정하기. EE의 green axis가 x축 이므로
    wrist_angle = math.atan2(y, x)
    self.angles = np.round(np.rad2deg(angle), 3)
    print(self.angles)

    return self.angles
   
def calc_ik(init_position, goal_position) : 
    arm = chain() 
    # calc_ik 함수 불러올때마다 클래스 새로 정의하기 때문에 나중에 수정 필요! main 함수에서 미리 클래서 정의해두기

    arm.angle_i = arm.IK(target_position=init_position)
    arm.angle_f = arm.IK(target_position=goal_position)

    trajectory_generation(arm.angle_i, arm.angle_f)

    return arm.angle_i, arm.angle_f 

def trajectory_generation(inital_angles, goal_angles) :
    
    d = [90, 35.25, 0, 0, 0]
    a = [0, 0, 158, 173, 73 + 51.25 + 50]
    al = [0, -90, 0, 0, -90]


    time = 50
    time_pause = 0.0005

    DOF = 5

    th_i = inital_angles 
    th_f = goal_angles

    th = np.zeros((time, DOF))

    for t in range(time):
        for i in range(DOF):
            th[t, i] = th_i[i] + 3 / (time**2) * (th_f[i] - th_i[i]) * t**2 - 2 / (time**3) * (th_f[i] - th_i[i]) * t**3

    P = np.zeros((time, DOF, 3))

    x = P[:, :, 0]
    y = P[:, :, 1]
    z = P[:, :, 2]

    for t in range(time):
        T = np.eye(4)
        for i in range(DOF):
            Trans = np.array([
                [np.cos(np.deg2rad(th[t, i])), -np.sin(np.deg2rad(th[t, i])) * np.cos(np.deg2rad(al[i])), np.sin(np.deg2rad(th[t, i])) * np.sin(np.deg2rad(al[i])), a[i] * np.cos(np.deg2rad(th[t, i]))],
                [np.sin(np.deg2rad(th[t, i])), np.cos(np.deg2rad(th[t, i])) * np.cos(np.deg2rad(al[i])), -np.cos(np.deg2rad(th[t, i])) * np.sin(np.deg2rad(al[i])), a[i] * np.sin(np.deg2rad(th[t, i]))],
                [0, np.sin(np.deg2rad(al[i])), np.cos(np.deg2rad(al[i])), d[i]],
                [0, 0, 0, 1]
            ])
            T = np.dot(T, Trans)
            P[t, i, :] = T[0:3, 3]

        msg = fl()
        msg.data = th[t,1:5]
        pub_angle = rospy.Publisher('angle', fl, queue_size=2)
        pub_angle.publish(msg)

        print(th[t,1:5])

        plt.pause(time_pause)

    x0 = P[time-1, DOF-1, 0]
    y0 = P[time-1, DOF-1, 1]
    z0 = P[time-1, DOF-1, 2]

    print('x={:.2f}, y={:.2f}, z={:.2f}'.format(x0, y0, z0))

#########################################################################################

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


class FSM:
    def __init__(self, sim_dt, obj_height):
        self._sim_dt = sim_dt
        self._obj_height = obj_height
        
        self._state_done = False
        self._state = "go_above_obj"
        self._last_state = "go_above_obj"
        
        self._above_offset = [0, 0, 0.05 + self._obj_height]
        self._grip_offset = [0, 0, 0.01 + self._obj_height]
        self._lift_offset = [0, 0, 0.10 + self._obj_height]
        
        self._hand_down_quat = [0.0, 0.0, -0.707107, 0.707107]

        grab_angle = np.pi / 6.0
        grab_axis = [0, 0, 1]
        grab_quat = quat_from_angle_axis(grab_angle, grab_axis)

        self._obj_grab_quat = quat_mul(grab_quat, self._hand_down_quat)
        
        self._gripper_open = False

        self.pub_goal_pose = rospy.Publisher('goal_pose', fl, queue_size=10)
        self.pub_grip_open = rospy.Publisher('grip_open', Bool, queue_size=10)

    def move(self, goal_pose):
        goal_msg = fl()
        goal_msg.data = goal_pose.tolist()
        self.pub_goal_pose.publish(goal_msg)
        rospy.loginfo('published - object pos(%.2f %.2f %.2f) ori(%.2f deg)', *goal_pose)

    def grip(self, grip_open):
        grip_msg = Bool()
        grip_msg.data = grip_open
        self.pub_grip_open.publish(grip_msg)
        rospy.loginfo('published - grip (%.2f)', grip_open)

    def new_state(self, data):
        rospy.loginfo('subscribed - new state = %r', data.data)
        self._state_done = data.data
        
        # 상태 완료 후 다음 동작으로 전환
        if self._state_done:
            self._state_done = False  # 다음 상태를 위해 초기화
            self.update(self._current_obj_pose)  # 현재 상태로 돌아가 다음 동작 수행

    def impact_feedback(self):
        rospy.loginfo('subscribed - impact detected! going back to last state')
        self._state = self._last_state
        self.update(self._current_obj_pose)
        rospy.loginfo('freeze for 10s')

    def update(self, obj_pose):
        self._current_obj_pose = obj_pose  # 현재 물체의 자세를 저장

        if self._state == "ungrip":
            self.grip(False)
            self._last_state = self._state
            self._state = "prep_grip"

        elif self._state == "go_above_obj":
            target_pose = obj_pose
            target_pose[:3] = target_pose[:3] + self._above_offset
            self.move(target_pose)
            self._last_state = self._state
            self._state = "grip"
            
        elif self._state == "prep_grip":
            target_pose = obj_pose
            target_pose[:3] = target_pose[:3] + self._grip_offset
            self.move(target_pose)
            self._last_state = self._state
            self._state = "grip"

        elif self._state == "grip":
            self.grip(True)
            self._last_state = self._state
            self._state = "done"
        
        if self._state == "done":
            rospy.loginfo("All tasks completed.")

def callback(data):
    rospy.loginfo('subscribed - object initial pos(%.2f %.2f %.2f) ori(%.2f deg), goal pos(%.2f %.2f %.2f) ori(%.2f deg)', *data.data)
    soomac_fsm.update(data.data)

def main():
    rospy.init_node('master', anonymous=True)
    global soomac_fsm
    soomac_fsm = FSM(1/60, 0.1)
    rospy.Subscriber('object_pose', fl, callback)
    rospy.Subscriber('state_done', Bool, soomac_fsm.new_state)
    rospy.Subscriber('impact_feedback', Bool, soomac_fsm.impact_feedback)
    rospy.Subscriber('task_type', fl, soomac_fsm.new_state)
    
    rospy.spin()

if __name__ == '__main__':
    try:
        rospy.logwarn("Master Node is on")
        main()
    except rospy.ROSInterruptException:
        rospy.loginfo('Program is shut down')
