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
rate = rospy.Rate(5)

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
rospy.Subscriber("gps", Pose2D, GPSCallback)

while not rospy.is_shutdown():
    if gotCallback:
        #point1 = path.poses[currentLocation]
        point2 = path.poses[currentLocation+2]

        calculated_x = (point2.pose.position.x - gpsX)/2
        calculated_y = (point2.pose.position.y - gpsY)/2

        hypeShit = math.sqrt(calculated_x*calculated_x + calculated_y*calculated_y)
        cosShit = math.acos(calculated_x/hypeShit)
        sinShit = math.asin(calculated_y/hypeShit)
 
        shit = cosShit + sinShit
        calculated_theta = cosShit - theta

        print calculated_theta, calculated_x, calculated_y

            
        # Create a message to publish
        cmd = Twist()

        cmd.linear.x = calculated_x
        cmd.linear.y = calculated_y
        cmd.angular.z = calculated_theta

        pub.publish(cmd)
        rate.sleep()
        counter = counter + 1
        if counter == 50:
            currentLocation=currentLocation+1
            counter = 0
