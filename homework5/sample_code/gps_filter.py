from math import *
import rospy
import tf
from geometry_msgs.msg import Pose2D, PoseStamped
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import numpy as np
from math import *



def GPSCallback(data):
    global gps#, xp, P, W

    gps[0] = data.x
    gps[1] = data.y
    gps[2] = data.theta

    '''y = gps - xp
            
                S = P + W
            
                kal = np.dot(P, np.linalg.inv(S))
            
                xp = xp + np.dot(kal,y)
            
                P = P - np.dot(kal,P)
            
                xp[0] += dd*(w1+w2)*cos(xp[2])
                xp[1] += dd*(w1+w2)*sin(xp[2])
                xp[2] += dd*(w1-w2)/L'''

    
    

def wheelCallback(msg):
    global w1, s2
    w1 = msg.points[0].velocities[1]
    w2 = msg.points[0].velocities[0]




rospy.init_node("filter")

#subscribe to the gps topic
sub = rospy.Subscriber("gps", Pose2D, GPSCallback)
sub = rospy.Subscriber("cmd_joint_traj", JointTrajectory, wheelCallback)


#publisher for filtered gps signal
pub = rospy.Publisher("gps_filter", Pose2D, queue_size = 10)
pub2 = rospy.Publisher("pose_estimate", PoseStamped, queue_size = 10)

rate = rospy.Rate(10)


dt = .00001
r = .05
dd = dt*r/2.
L = .075

sigma1 = 0.2
var1 = sigma1**2
var2 = .2*.2

w1 = 0.
w2 = 0.

x = np.zeros(3)

V = np.array([[.0001,0,0],[0,.0001,0],[0,0,.0001]])
W = np.array([[var2,0,0],[0,var2,0],[0,0,var2]])
P = np.zeros((3,3))
xp = np.zeros(3)

H = np.array([[1,0,0],[0,1,0],[0,0,1]])
HT = H.T

gps = np.zeros(3)

w1 = 0.
w2 = 0.

xp = np.zeros(3)
xf = np.zeros(3)








while not rospy.is_shutdown():
    xp[0] = xf[0] + dd*(w1+w2)*cos(xf[2])
    xp[1] = xf[1] + dd*(w1+w2)*sin(xf[2])
    xp[2] = xf[2] + dd*(w1-w2)/L

    F1 = [1.,0,-dd*(w1+w2)*sin(xf[2])]
    F2 = [0,1., dd*(w1+w2)*cos(xf[2])]
    F = np.array([F1,F2,[0,0,1.]])
    FT = F.T

    pp = np.dot(F, np.dot(P,FT)) + V

    y = gps - np.dot(H,xp)

    S = np.dot(H, np.dot(pp,HT)) + W
    SI = np.linalg.inv(S)

    kal = np.dot(pp, np.dot(HT,SI))

    xf = xp + np.dot(kal,y)

    P = pp - np.dot(kal, np.dot(H,pp))

    gpsfil = Pose2D()
    gpsfil.x = xf[0]
    gpsfil.y = xf[1]
    gpsfil.theta = xf[2]

    pub.publish(gpsfil)

    estPose = PoseStamped()
    estPose.header.frame_id = "map"
    estPose.pose.position.x = xf[0]
    estPose.pose.position.y = xf[1]
    quaternion = tf.transformations.quaternion_from_euler(0,0,xf[2])
    estPose.pose.orientation.x = quaternion[0]
    estPose.pose.orientation.y = quaternion[1]
    estPose.pose.orientation.z = quaternion[2]
    estPose.pose.orientation.w = quaternion[3]
    pub2.publish(estPose)
    #rate.sleep()


   