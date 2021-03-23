#! /usr/bin/python
# -*- coding: utf-8 -*-
import rospy
from std_msgs.msg import Float32MultiArray
def talker():
    pub_p = rospy.Publisher('lefttop_point', Float32MultiArray, queue_size=1)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        array = [521,1314]
        left_top = Float32MultiArray(data=array)
            # can also be assigned in the following form
        #left_top = Float32MultiArray()
        #left_top.data = [521,1314]
        #left_top.label = 'love'
        rospy.loginfo(left_top)
        pub_p.publish(left_top)
        rate.sleep()
 
if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass