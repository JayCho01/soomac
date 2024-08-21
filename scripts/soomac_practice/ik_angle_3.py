# Pick and Place 위한 IK : 초기점과 목표점 받아와서 IK 계산 후 각도 값들 Publish

import rospy
import numpy as np
import matplotlib.pyplot as plt
from ikpy.chain import Chain
from ikpy.link import OriginLink, URDFLink
from std_msgs.msg import Float32MultiArray as fl


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
    angle = self.arm.inverse_kinematics(target_position, target_orientation=[0, 0, -1], orientation_mode="X")
    # orientation mode 를 "X"로 설정하기. EE의 green axis가 x축 이므로.
    self.angles = np.round(np.rad2deg(angle), 3)
    print(self.angles)

    return self.angles
  
# ###################################
 
def calc_ik(goal_position) : 
    arm = chain()

    arm.angle_i = arm.IK(target_position=init_position)
    arm.angle_f = arm.IK(target_position=goal_position)

    trajectory_generation(arm.angle_i, arm.angle_f)

    return arm.angle_i, arm.angle_f 


# ###########################################################################

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


######################################################################

def callback1(data):
    
    global init_position 
    
    init_position = np.array(data.data)

    rospy.loginfo("initial position is %.2f %.2f %.2f", *init_position)
  
    return init_position


def callback2(data):
    
    global init_position

    # Ensure init_position is initialized
    if init_position is None:
        rospy.logwarn("Initial position not yet received!")
        return
    
    goal_position = np.array(data.data)

    rospy.loginfo("goal position is %.2f %.2f %.2f", *goal_position)

    angle_i, angle_f = calc_ik(data.data)
    
    rospy.loginfo(angle_i)
    rospy.loginfo(angle_f)

    

  
################################################################

def main():
  
    global init_position
    init_position = [79, 79 , 200]
    
    rospy.init_node('ik_angle', anonymous=True)
    rospy.Subscriber("initial_position", fl, callback1)
    rospy.Subscriber("goal_position", fl, callback2)

    rospy.spin()


#############################################################################3

if __name__ == '__main__':
    try:
        rospy.logwarn("ik_angle Node is on")
        main()
    except rospy.ROSInterruptException:
        print('program is shut downed')