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

    point1 = data.poses[0]
    point2 = data.poses[1]

    calculated_x = point2.pose.position.x - point1.pose.position.x
    calculated_y = point2.pose.position.y - point1.pose.position.y
    if (calculated_x != 0):
        calculated_theta = math.atan(calculated_y/calculated_x) - theta
    elif(calculated_y > 0):
        calculated_theta = 1.5707 - theta
    elif(calculated_y < 0):
        calculated_y = -1.5707 - theta

    # Create a message to publish
    cmd = Twist()

    cmd.linear.x = calculated_x
    cmd.linear.y = calculated_y
    cmd.angular.z = calculated_theta

    pub.publish(cmd)
    rate.sleep()

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

