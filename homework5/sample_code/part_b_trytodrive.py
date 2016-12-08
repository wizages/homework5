#!/usr/bin/env python
import rospy
import math
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
    global gotCallback, path, currentLocation, counter
    gotCallback = True
    path = data
    currentLocation = 0
    counter = 0

    

# Start a ROS node.
rospy.init_node('driver')
rate = rospy.Rate(8)

gpsX = 0
gpsY = 0
theta = 0
gotCallback = False
currentLocation = 0
couter = 0

path = Path()

# Wheel velocity publisher
pub = rospy.Publisher("cmd_vel", Twist, queue_size=10)

rospy.Subscriber("path", Path, callback)
rospy.Subscriber("gps_filter", Pose2D, GPSCallback)

while not rospy.is_shutdown():
    if gotCallback:
        #point1 = path.poses[currentLocation]
        point2 = path.poses[currentLocation+4]

        calculated_x = (point2.pose.position.x - gpsX)/8.
        calculated_y = (point2.pose.position.y - gpsY)/8.

        hypeShit = math.sqrt(calculated_x*calculated_x + calculated_y*calculated_y)
        cosShit = math.acos(calculated_x/hypeShit)
        sinShit = math.asin(calculated_y/hypeShit)
        tanShit = math.atan2(calculated_y,calculated_x)
 
        shit = cosShit + sinShit


        if (tanShit > 0 and theta <= math.pi):
            calculated_theta = tanShit - theta
        else:
            calculated_theta = tanShit - theta

        if(abs(calculated_theta) > math.pi/2):
            calculated_theta = -tanShit + theta



        print theta, tanShit, calculated_theta

            
        # Create a message to publish
        cmd = Twist()

        cmd.linear.x = calculated_x
        cmd.linear.y = calculated_y
        cmd.angular.z = calculated_theta

        pub.publish(cmd)
        rate.sleep()
        counter = counter + 1
        if counter == 10:
            currentLocation=currentLocation+1
            counter = 0
