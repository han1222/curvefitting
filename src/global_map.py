#!/usr/bin/env python

# Python includes
import numpy as np
import random

# ROS includes
import rospy
from geometry_msgs.msg import Pose, Point
import rviz_tools_py as rviz_tools

data = np.loadtxt('/home/han/catkin_ws/src/curve_fitting/src/global_waypoint.txt',dtype=np.float32)
x = data[:,0]
y = data[:,1]
print(x,y)
# Initialize the ROS Node
rospy.init_node('global_map', anonymous=False, log_level=rospy.INFO, disable_signals=False)

# Define exit handler
# def cleanup_node():
#     markers.deleteAllMarkers()

# rospy.on_shutdown(cleanup_node)

markers = rviz_tools.RvizMarkers('/odom', 'glboal_map')

while not rospy.is_shutdown():
    # Path:

    # Publish a path using a list of ROS Point Msgs
    path = []
    for x_i,y_i in zip(x,y):
        path.append(Point(x_i,y_i,0))
    width = 1
    markers.publishPath(path, 'orange', width, 5.0) # path, color, width, lifetime


    rospy.Rate(1).sleep() #1 Hz

