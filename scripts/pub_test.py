import rospy
from std_msgs.msg import Int32, String, Float32, Bool
from std_msgs.msg import Float32MultiArray as fl
import numpy as np


class pub_node():
    def __init__(self):
        rospy.init_node('pub_test', anonymous=True)
        self.pos = np.array([100, 100, 100, 30, 5, 200, 200, 200, 50])
        self.main()

    def main(self):
        rate = rospy.Rate(10) # 10hz

        while not rospy.is_shutdown():
            number = input("pub 종류 : ") # 숫자 입력 받음
            number = int(number)
            if number == 0:
                self.vision()

            elif number == 1:
                self.gui_start()

            elif number == 2:
                self.gui_stop()

            elif number == 3:
                self.state_done()
            rate.sleep()

    def vision(self):
        print('vision topic')
        msg = fl()
        msg.data = self.pos
        pub_vision.publish(msg)

    def gui_start(self):
        print('gui : start')
        msg = Bool()
        msg.data = True
        pub_gui_start.publish(msg)

    def gui_stop(self):
        print('gui : stop')
        msg = Bool()
        msg.data = True
        pub_gui_stop.publish(msg)

    def state_done(self):
        print('state_done topic')
        msg = Bool()
        msg.data = True
        pub_state_done.publish(msg)        

if __name__ == '__main__':
    try:
        pub_vision = rospy.Publisher('vision', fl, queue_size=10) 
        pub_gui_start = rospy.Publisher('gui_start', Bool, queue_size=10)
        pub_gui_stop = rospy.Publisher('gui_stop', Bool, queue_size=10)
        pub_state_done = rospy.Publisher('state_done', Bool, queue_size=10)

        pub_node()
    except rospy.ROSInterruptException:
        print('program is shut downed')    
