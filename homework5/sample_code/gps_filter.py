from math import *
import rospy
import tf
from geometry_msgs.msg import Pose2D, PoseStamped
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import numpy as np
from math import *


#publisher for filtered gps signal
pub = rospy.Publisher("gps_filter", Pose2D, queue_size = 10)
pub2 = rospy.Publisher("pose_estimate", PoseStamped, queue_size = 10)

def GPSCallback(data):
    global gps, xp, P, W

    gps[0] = data.x
    gps[1] = data.y
    gps[2] = data.theta

    y = gps - xp

    S = P + W

    kal = np.dot(P, np.linalg.inv(S))

    xp = xp + np.dot(kal,y)

    P = P - np.dot(kal,P)

    gpsfil = Pose2D()
    gpsfil.x = xp[0]
    gpsfil.y = xp[1]
    gpsfil.theta = xp[2]

    pub.publish(gpsfil)

    estPose = PoseStamped()
    estPose.header.frame_id = "map"
    estPose.pose.position.x = xp[0]
    estPose.pose.position.y = xp[1]
    quaternion = tf.transformations.quaternion_from_euler(0,0,xp[2])
    estPose.pose.orientation.x = quaternion[0]
    estPose.pose.orientation.y = quaternion[1]
    estPose.pose.orientation.z = quaternion[2]
    estPose.pose.orientation.w = quaternion[3]
    pub2.publish(estPose)
    rate.sleep()
    

def wheelCallback(msg):
    global w1, s2
    w1 = msg.points.velocities[1]
    w2 = msg.points.velocities[0]


sigma1 = 0.2
var1 = sigma1**2
var2 = .001

w1 = 0.
w2 = 0.

x = np.zeros(3)

V = np.array([[var1,0,0],[0,var1,0],[0,0,var1]])
W = np.array([[var2,0,0],[0,var2,0],[0,0,var2]])
P = W
xp = np.zeros(3)

gps = np.zeros(3)

w1 = 0.
w2 = 0.




rospy.init_node("filter")

#subscribe to the gps topic
sub = rospy.Subscriber("gps", Pose2D, GPSCallback)
sub = rospy.Subscriber("cmd_joint_traj", JointTrajectory, wheelCallback)



rate = rospy.Rate(10)

while not rospy.is_shutdown():
    pass
   