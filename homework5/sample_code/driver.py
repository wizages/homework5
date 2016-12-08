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
    global gpsY, gpsX

    point1 = data.poses[0]
    point2 = data.poses[3]

    calculated_x = (point2.pose.position.x - gpsX)/3
    calculated_y = (point2.pose.position.y - gpsY)/3

    hypeShit = math.sqrt(calculated_x*calculated_x + calculated_y*calculated_y)
    cosShit = math.acos(calculated_x/hypeShit)
    sinShit = math.asin(calculated_y/hypeShit)

    shit = cosShit + sinShit

    if calculated_y == 0 and calculated_x > 0:
        calculated_theta = 0 - theta
    elif calculated_y == 0 and calculated_x < 0:
        calculated_theta = 3.14159 - theta
    elif calculated_x == 0 and calculated_y > 0:
        calculated_theta = 3.14159/2 - theta
    elif calculated_x == 0 and calculated_y < 0:
        calculated_theta = 3*3.14159/2 - theta
    else:
        calculated_theta = math.atan(calculated_y/calculated_x) - theta

    print calculated_theta, calculated_x, calculated_y

        
    # Create a message to publish
    cmd = Twist()

    cmd.linear.x = calculated_x
    cmd.linear.y = calculated_y
    cmd.angular.z = calculated_theta

    pub.publish(cmd)
    #rate.sleep()

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

