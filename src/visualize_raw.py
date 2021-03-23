#!/usr/bin/env python

# Python includes


# ROS includes
import roslib
import rospy
import numpy as np
from nav_msgs.msg import Path
from geometry_msgs.msg import Pose, Point, Quaternion, Vector3, Polygon
from tf import transformations # rotation_matrix(), concatenate_matrices()
import rviz_tools_py as rviz_tools

#rospy.init_node('test', anonymous=False, log_level=rospy.INFO, disable_signals=False)
rospy.init_node('before_fitting', anonymous=True)
# Define exit handler
def cleanup_node():
    print "Shutting down node"
    markers.deleteAllMarkers()

rospy.on_shutdown(cleanup_node)
markers = rviz_tools.RvizMarkers('/base_link', 'visualization_marker')

from nav_msgs.msg import Path

   
def pathCallback(msg):
    x=[]
    y=[]
    path = []
    for i in range (0,50):
        x.append(msg.poses[i].pose.position.x)
        y.append(msg.poses[i].pose.position.y)
       
        path.append( Point(msg.poses[i].pose.position.x,
        msg.poses[i].pose.position.y,0) )
    print(path)
    # Path:
    # Publish a path using a list of ROS Point Msgs
    
    width = 0.09
    markers.publishPath(path, 'orange', width, 0.05 ) # path, color, width, lifetime
    

if __name__== '__main__':

    rospy.Subscriber("Path/LocalWaypoint/OnBody", Path, pathCallback)
    rospy.spin() 
