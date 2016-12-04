#!/usr/bin/env python
import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Twist
import math

def callback(msg):
    xd = msg.linear.x
    yd = msg.linear.y
    thd = msg.linear.z
    v = math.sqrt(xd**2 + yd**2)

    phi2 = (-L*thd)/r + v/r
    phi1 = (L*thd)/r + v/r

    # Create a message to publish
    cmd = JointTrajectory()

    # Add joint names for our left and right wheels
    cmd.joint_names.append("left")
    cmd.joint_names.append("right")

    # Add our wheel velocities (radians/sec)
    p = JointTrajectoryPoint()
    p.velocities.append(phi2) # left wheel
    p.velocities.append(phi1) # right wheel
    cmd.points = [p]

    # Publish our wheel velocities
    pub.publish(cmd)

L = .15/2.
r = .05


# Start a ROS node.
rospy.init_node('diff_drive')
# Wheel velocity publisher
pub = rospy.Publisher("cmd_joint_traj", JointTrajectory, queue_size=10)
# Rate to publish the wheel velocities
rate = rospy.Rate(10) # 10 hz
#Subscribe to the cmd_vel topic
rospy.Subscriber("cmd_vel", Twist, callback)

# Loop until we shut the node down (control-c).
while not rospy.is_shutdown():
    pass


    '''# Create a message to publish
    cmd = JointTrajectory()

    # Add joint names for our left and right wheels
    cmd.joint_names.append("left")
    cmd.joint_names.append("right")

    # Add our wheel velocities (radians/sec)
    p = JointTrajectoryPoint()
    p.velocities.append(1.0) # left wheel
    p.velocities.append(1.0) # right wheel
    cmd.points = [p]

    # Publish our wheel velocities
    pub.publish(cmd)
    rate.sleep()'''
