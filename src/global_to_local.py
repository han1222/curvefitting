#!/usr/bin/env python
import math
import numpy as np
import rospy
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Float32 
from nav_msgs.msg import Odometry
from nav_msgs.msg import Path
from geometry_msgs.msg import Pose, Point,Vector3, PoseStamped
import rviz_tools_py as rviz_tools

rospy.init_node('global_to_local', anonymous=True)


data = np.loadtxt('/home/han/catkin_ws/src/curve_fitting/src/global_waypoint.txt',dtype=np.float32)

translation=np.empty((0,2))
map_x = data[:,0]
map_y = data[:,1]
map_x=np.array(map_x) #2507
map_y=np.array(map_y)
g_wpt=np.c_[map_x,map_y]

def conversion_cb(msg):
    distance=[]
    ## quaternions to euler
    x = msg.pose.pose.orientation.x
    y = msg.pose.pose.orientation.y
    z = msg.pose.pose.orientation.z
    w = msg.pose.pose.orientation.w
    ysqr = y * y

    # t0 = +2.0 * (w * x + y * z)
    # t1 = +1.0 - 2.0 * (x * x + ysqr)
    # X = math.degrees(math.atan2(t0, t1))

    # t2 = +2.0 * (w * y - z * x)
    # t2 = +1.0 if t2 > +1.0 else t2
    # t2 = -1.0 if t2 < -1.0 else t2
    # Y = math.degrees(math.asin(t2))
    
    # Yaw angle
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (ysqr + z * z)
    Z = math.degrees(math.atan2(t3, t4))
    ## quaternions to euler

    theta = np.radians(Z) 
    c, s = np.cos(theta), np.sin(theta)
    R = np.array(((c, s), (-s, c)))
    for i in range(len(map_x)):
        distance.append(math.sqrt(math.pow(map_x[i]-msg.pose.pose.position.x,2)+math.pow(map_y[i]-msg.pose.pose.position.y,2)))      
   
    b_wpt=np.empty((0,2))
    b_wpt_tmp=np.empty((0,2))
    for i in range(distance.index(min(distance)),len(g_wpt)):
       b_wpt_tmp = np.array([np.matmul(R,g_wpt[i])])
       translation=np.array([-msg.pose.pose.position.x*c-msg.pose.pose.position.y*s, msg.pose.pose.position.x*s-msg.pose.pose.position.y*c])
       b_wpt_tmp = np.add(b_wpt_tmp,translation)
       
       b_wpt=np.append(b_wpt,b_wpt_tmp[0])
    
    b_wpt=b_wpt.flatten()
    b_wpt=b_wpt.tolist()
    b_wpt_pub=Float32MultiArray(data=b_wpt)
    pub.publish(b_wpt_pub)

    # ######## pub path ########
    # path_msg     = Path()
    # path_msg.header.stamp = rospy.Time.now()
    # path_msg.header.frame_id = 'base_link'
    # for i in range(0,len(b_wpt)/2):
    #     pose_stamped = PoseStamped()
    #     pose_stamped.header.frame_id = 'base_link'
    #     pose_stamped.pose.position.x = b_wpt[2*i]
    #     pose_stamped.pose.position.y = b_wpt[2*i+1]
    #     pose_stamped.pose.orientation.w = 1
    #    # print("(x, y):", pose_stamped.pose.position.x, pose_stamped.pose.position.y)
    #     path_msg.poses.append(pose_stamped)
    # pubPath_sw.publish(path_msg)



if __name__== '__main__':
    rospy.Subscriber('/simulation/bodyOdom',Odometry,conversion_cb)
    pub=rospy.Publisher('/Point/LocalWaypoint/OnBody',Float32MultiArray,queue_size=10)
    # pubPath_sw=rospy.Publisher('/sw/path',Path,queue_size=10)
    rospy.spin() 
