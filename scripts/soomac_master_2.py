#!/usr/bin/env python

import rospy
import numpy as np
from std_msgs.msg import Float32MultiArray as fl
from std_msgs.msg import Bool


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
        
        self._above_offset = [0, 0, 0.05 + self._obj_height]
        self._grip_offset = [0, 0, 0.01 + self._obj_height]
        self._lift_offset = [0, 0, 0.10 + self._obj_height]
        
        self._hand_down_quat = [0.0, 0.0, -0.707107, 0.707107]

        grab_angle = np.pi / 6.0
        grab_axis = [0, 0, 1]
        grab_quat = quat_from_angle_axis(grab_angle, grab_axis)

        self._obj_grab_quat = quat_mul(grab_quat, self._hand_down_quat)
        
        self._gripper_separation = 0.0
        self._gripper_open = 0.4
        self._gripper_close = 0.0

        self.pub_goal_pose = rospy.Publisher('goal_pose', fl, queue_size=10)
        self.pub_grip_deg = rospy.Publisher('grip_deg', fl, queue_size=10)

    def move(self, goal_pose):
        goal_msg = fl()
        goal_msg.data = goal_pose.tolist()
        self.pub_goal_pose.publish(goal_msg)
        rospy.loginfo('published - object pos(%.2f %.2f %.2f) ori(%.2f %.2f %.2f %.2f)', *goal_pose)

    def grip(self, grip_deg):
        grip_msg = fl()
        grip_msg.data = [grip_deg]
        self.pub_grip_deg.publish(grip_msg)
        rospy.loginfo('published - gripper degree (%.2f)', grip_deg)

    def new_state(self, data):
        rospy.loginfo('subscribed - new state = %r', data.data)
        self._state_done = data.data
        
        # 상태 완료 후 다음 동작으로 전환
        if self._state_done:
            self._state_done = False  # 다음 상태를 위해 초기화
            self.update(self._current_obj_pose)  # 현재 상태로 돌아가 다음 동작 수행

    def update(self, obj_pose):
        self._current_obj_pose = obj_pose  # 현재 물체의 자세를 저장

        if self._state == "go_above_obj":
            target_pose = obj_pose
            target_pose[:3] = target_pose[:3] + self._above_offset
            self.move(target_pose)
            self.grip(self._gripper_open)
            self._state = "prep_grip"
            
        elif self._state == "prep_grip":
            target_pose = obj_pose
            target_pose[:3] = target_pose[:3] + self._grip_offset
            self.move(target_pose)
            self.grip(self._gripper_open)
            self._state = "grip"

        elif self._state == "grip":
            target_pose = obj_pose
            target_pose[:3] = target_pose[:3] + self._grip_offset
            self.move(target_pose)
            self.grip(self._gripper_close)
            self._state = "done"
        
        if self._state == "done":
            rospy.loginfo("All tasks completed.")

def callback(data):
    rospy.loginfo('subscribed - object initial pos (%.2f %.2f %.2f) ori(%.2f %.2f %.2f %.2f), goal pos (%.2f %.2f %.2f) ori(%.2f %.2f %.2f %.2f)', *data.data)
    soomac_fsm.update(data.data)

def main():
    rospy.init_node('master', anonymous=True)
    global soomac_fsm
    soomac_fsm = FSM(1/60, 0.1)
    rospy.Subscriber('object_pose', fl, callback)
    rospy.Subscriber('state_done', Bool, soomac_fsm.new_state)
    
    rospy.spin()

if __name__ == '__main__':
    try:
        rospy.logwarn("Master Node is on")
        main()
    except rospy.ROSInterruptException:
        rospy.loginfo('Program is shut down')
