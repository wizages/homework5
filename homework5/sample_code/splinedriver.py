#!/usr/bin/env python
import rospy
import math
import numpy as np
from geometry_msgs.msg import Twist
from nav_msgs.msg import Path
from geometry_msgs.msg import Pose2D, PoseStamped
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

def GPSCallback(msg):
    global gpsX, gpsY, theta
    gpsX = msg.x
    gpsY = msg.y
    theta = msg.theta

def callback(data):
    global gpsX, gpsY

    t0, t1 = 0, 1
    x0 = gpsX
    x1 = data.poses[3].pose.position.x
    x2 = data.poses[4].pose.position.x
    y0 = gpsY
    y1 = data.poses[3].pose.position.y
    y2 = data.poses[4].pose.position.y

    xd0 = x1-x0
    yd0 = y1-y0
    xd1 = x2-x1
    yd1 = y2-y1

    dt = 1

    dx = x1 - x0
    dy = y1 - y0

    a = xd0*dt - dx
    b = -xd1*dt + dx

    c = yd0*dt - dy
    d = -yd1*dt + dy

    t = np.linspace(t0,t1,10)

    dotz = 1./dt

    z = (dotz)*(t-t0)
    x = (1-z)*x0 + z*x1+z*(1-z)*(a*(1-z)+b*z)
    y = (1-z)*y0 + z*y1+z*(1-z)*(c*(1-z)+d*z)

    for i in range(0,len(x)-1):
        startx = x[i]
        endx = x[i+1]
        starty = y[i]
        endy = y[i+1]
        calculated_x = endx - startx
        calculated_y = endy - starty

        hypeShit = math.sqrt(calculated_x*calculated_x + calculated_y*calculated_y)
        cosShit = math.acos(calculated_x/hypeShit)
        sinShit = math.asin(calculated_y/hypeShit)
        
        shit = cosShit + sinShit
        
        calculated_theta = cosShit - theta

        # Create a message to publish
        cmd = Twist()

        cmd.linear.x = calculated_x
        cmd.linear.y = calculated_y
        cmd.angular.z = calculated_theta

        pub.publish(cmd)
        rate.sleep()


    '''point1 = data.poses[1]
                point2 = data.poses[5]
            
                calculated_x = (point2.pose.position.x - point1.pose.position.x)
                calculated_y = (point2.pose.position.y - point1.pose.position.y)
            
                hypeShit = math.sqrt(calculated_x*calculated_x + calculated_y*calculated_y)
                cosShit = math.acos(calculated_x/hypeShit)
                sinShit = math.asin(calculated_y/hypeShit)
            
                shit = cosShit + sinShit
            
                calculated_theta = cosShit - theta
            
                print calculated_theta, calculated_x, calculated_y'''

        


# Start a ROS node.
rospy.init_node('driver')
rate = rospy.Rate(10)

gpsX = 0
gpsY = 0
theta = 0

# Wheel velocity publisher
pub = rospy.Publisher("cmd_vel", Twist, queue_size=10)

rospy.Subscriber("path", Path, callback)
rospy.Subscriber("gps", Pose2D, GPSCallback)

rospy.spin()

