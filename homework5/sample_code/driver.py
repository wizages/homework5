#!/usr/bin/env python
import rospy
import math
from geometry_msgs.msg import Twist
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

# Start a ROS node.
rospy.init_node('driver')
# Wheel velocity publisher
pub = rospy.Publisher("cmd_vel", Twist, queue_size=10)

rospy.Subscriber("path", Path, callback)

rospy.spin()

def callback(data)

    point1 = data.poses[0]
    point2 = data.poses[1]

    calculated_x = point2.pose.position.x - point1.pose.position.x
    calculated_y = point2.pose.position.y - point1.pose.position.y
    calculated_theta = math.atan(calculated_y/calculated_x)

    # Create a message to publish
    cmd = Twist()

    twist.linear.x = calculated_x
    twist.linear.y = calculated_y
    twist.angular.z = calculated_theta

    pub.publish(cmd)
    rate.sleep()