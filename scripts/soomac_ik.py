import rospy
import numpy as np
from geometry_msgs.msg import Pose
from std_msgs.msg import Float32MultiArray

class InverseKinematicsNode:
    def __init__(self):
        rospy.init_node('inverse_kinematics_node', anonymous=True)
        
        self.pub_goal_pose = rospy.Publisher('goal_pose', Float32MultiArray, queue_size=10)
        
        self.timer = rospy.Timer(rospy.Duration(0.1), self.calculate_and_publish)

    def inverse_kinematics(self, target_pose):
        goal_pose = np.zeros(7)
        goal_pose[:3] = [target_pose.position.x, target_pose.position.y, target_pose.position.z]
        goal_pose[3:] = [target_pose.orientation.w, target_pose.orientation.x, 
                         target_pose.orientation.y, target_pose.orientation.z]
        
        return goal_pose

    def calculate_and_publish(self, event):
     
        target_pose = Pose()
        target_pose.position.x = 0.5
        target_pose.position.y = 0.2
        target_pose.position.z = 0.1
        
       
        target_pose.orientation.w = 1.0
        target_pose.orientation.x = 0.0
        target_pose.orientation.y = 0.0
        target_pose.orientation.z = 0.0
        
        
        goal_pose = self.inverse_kinematics(target_pose)
        
       
        goal_pose_msg = Float32MultiArray()
        goal_pose_msg.data = goal_pose
        
      
        self.pub_goal_pose.publish(goal_pose_msg)
        rospy.loginfo("Published goal pose: position(%.2f, %.2f, %.2f) orientation(%.2f, %.2f, %.2f, %.2f)", 
                      *goal_pose)

if __name__ == '__main__':
    try:
       
        ik_node = InverseKinematicsNode()
        rospy.loginfo("Inverse Kinematics Node Started")
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Inverse Kinematics Node Terminated")