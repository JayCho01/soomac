#!/usr/bin/env python

import rospy
import numpy as np
from isaacgym.torch_utils import quat_from_angle_axis, quat_mul
from std_msgs.msg import Float32MultiArray as fl

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
        grab_quat = quat_from_angle_axis(grab_angle, grab_axis).squeeze()

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

    def update(self, obj_pose):
        newState = self._state

        if self._state == "go_above_obj":
            target_pose = obj_pose
            target_pose[:3] = target_pose[:3] + self._above_offset
            self.move(target_pose)
            self.grip(self._gripper_open)
            if self._state_done:
                newState = "prep_grip"

        elif self._state == "prep_grip":
            target_pose = obj_pose
            target_pose[:3] = target_pose[:3] + self._grip_offset
            self.move(target_pose)
            self.grip(self._gripper_open)
            if self._state_done:
                newState = "grip"

        elif self._state == "grip":
            target_pose = obj_pose
            target_pose[:3] = target_pose[:3] + self._grip_offset
            self.move(obj_pose)
            self.grip(self._gripper_close)
            if self._state_done:
                newState = "lift"

        if newState != self._state:
            self._state = newState
            self._state_done = False  # Reset state_done for the next state transition
            print(f"Going to state {newState}")

def callback(data):
    rospy.loginfo('subscribed - object pos(%.2f %.2f %.2f) ori(%.2f %.2f %.2f %.2f)', *data.data)
    soomac_fsm.update(data.data)

def new_state(data):
    rospy.loginfo('subscribed - new state = %r', data.data)
    soomac_fsm._state_done = data.data

def main():
    rospy.init_node('master', anonymous=True)
    global soomac_fsm
    soomac_fsm = FSM(1/60, 0.1)
    rospy.Subscriber('object_pose', fl, callback)
    rospy.Subscriber('state_done', bool, new_state)
    rospy.spin()

if __name__ == '__main__':
    try:
        rospy.logwarn("Master Node is on")
        main()
    except rospy.ROSInterruptException:
        rospy.loginfo('Program is shut down')
