#!/usr/bin/env python
import roslib
import rospy
import numpy as np
from std_msgs.msg import Float32MultiArray
from nav_msgs.msg import Path

rospy.init_node('send_coeffcient', anonymous=True)

def pathCallback (msg):
    x=[]
    y=[]                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                    
    for i in range (0,len(msg.poses)):
        x.append(msg.poses[i].pose.position.x)
        y.append(msg.poses[i].pose.position.y)
    coefs = np.polyfit(x, y, 3)
    coefficients=Float32MultiArray(data=coefs)
    pub.publish(coefficients)
    #rospy.loginfo(coefficients)

if __name__== '__main__':  

    rospy.Subscriber("/Path/LocalWaypoint/OnBody", Path, pathCallback)
    pub=rospy.Publisher('coefficients',Float32MultiArray,queue_size=10)
    rospy.spin()
    
    
   
