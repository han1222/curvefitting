#!/usr/bin/env python

import roslib
import rospy
import numpy as np
import matplotlib.pyplot as plt
from std_msgs.msg import Float32MultiArray
from nav_msgs.msg import Path
from std_msgs.msg import Int32
from geometry_msgs.msg import Pose, Point
import rviz_tools_py as rviz_tools

rospy.init_node('polyfitting', anonymous=True)
markers = rviz_tools.RvizMarkers('/base_link', 'visualization_marker2')

def pathCallback (msg):  
    x=[]
    y=[]
    new_y=[]
    path = []
    #print(len(msg.poses)                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                        
    for i in range (0,len(msg.poses)):
        x.append(msg.poses[i].pose.position.x)
        y.append(msg.poses[i].pose.position.y)
    coefs = np.polyfit(x, y, 3)
    coefficients=Float32MultiArray(data=coefs)
    rospy.loginfo(coefficients)
    pub2.publish(coefficients)
    ffit = np.polyval(coefs, x)
    for element in ffit:
        new_y.append(element)
    for x_i,y_i in zip(x,new_y):
        path.append(Point(x_i,y_i,0))
    width = 0.09
    markers.publishPath(path, 'blue', width, 0.05) # path, color, width, lifetime
if __name__== '__main__':  
    pub2=rospy.Publisher('coefficients',Float32MultiArray,queue_size=10)
    rospy.Subscriber("Path/LocalWaypoint/OnBody", Path, pathCallback)
    rospy.spin()
