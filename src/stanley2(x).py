#!/usr/bin/env python
# I will add feedforwardterm later
# high speed damping term too
import rospy
import math
from sympy import Derivative, symbols
from geometry_msgs.msg import Pose,Point
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Float32
global crosstrack_error_term 
global heading_error_term
global cte
velocity=10
max_theta=0.523598 #I added margin
min_theta=-0.523598
soft_term=0.01

# Please change gain here for your system
gain=0.1

rospy.init_node('stanley')

def crosstrack_error(msg):
    global cte
    global crosstrack_error_term
    crosstrack_error_term=math.atan((gain*cte)/(velocity+soft_term))

    #rospy.loginfo(crosstrack_error_term)

def heading_error(msg):
    global heading_error_term
    global cte
    cte =msg.data[0]
    x = symbols('x')
     #add y value: Pleas change order here  if it doesn't work for example [0]->[1]->[2]->[3] 
    fx = msg.data[3] * x ** 3 + msg.data[2] * x ** 2 +  msg.data[1] * x ** 1 + msg.data[0]
    fprime = Derivative(fx, x).doit()
    n = fprime.subs({x: 0}) #now x  not 3  3 is temporary value
    heading_error_term=math.atan(n)
    #print("heading_error term: ",heading_error_term )
    #print("crosstrack_error_term: ",crosstrack_error_term)
    delta_command()

#[rad]
def delta_command():
    global crosstrack_error_term
    global heading_error_term
    #print("crosstrack_error_term: ",crosstrack_error_term)
    #print("heading_error_term: ",heading_error_term)
    delta= crosstrack_error_term+ heading_error_term
    if max_theta < delta:
        delta=max_theta
    elif delta< min_theta:
        delta=min_theta
    print("delta: ",delta)
    pub.publish(delta)
   
if __name__== '__main__':
    pub=rospy.Publisher('delta_wheel',Float32)
    sub=rospy.Subscriber('poly_path',Point,crosstrack_error)
    sub2=rospy.Subscriber('coefficients',Float32MultiArray,heading_error)
    rospy.spin() 
