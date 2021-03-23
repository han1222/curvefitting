#!/usr/bin/env python
import numpy as np
import rospy
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Float32 
from nav_msgs.msg import Odometry
from nav_msgs.msg import Path
from geometry_msgs.msg import Pose, Point
import rviz_tools_py as rviz_tools

rospy.init_node('global', anonymous=True)
markers = rviz_tools.RvizMarkers('/odom', 'Onbody')


theta = np.radians(90)
c, s = np.cos(theta), np.sin(theta)
R = np.array(((c, -s), (s, c)))
data = np.loadtxt('global_waypoint.txt')
b_wpt=np.empty((0,2))
translation=np.empty((0,2))
x = data[:,0]
y = data[:,1]
x=np.array(x)
y=np.array(y)
g_wpt=np.c_[x,y]

for i in range(0,len(g_wpt)):
    b_wpt=np.append(b_wpt,np.array([np.matmul(g_wpt[i],R)]),axis=0)
    translation=np.array([0,0])
    b_wpt=np.add(b_wpt,translation)

#b_wpt=b_wpt.tolist()
#width = 0.09
#markers.publishPath(b_wpt, 'blue', width, 0.05) # path, color, width, lifetime
