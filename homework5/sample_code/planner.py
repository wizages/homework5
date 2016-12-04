import rospy
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import Pose2D
import math

def GPSCallback(msg):
    global gpsX, gpsY
    gpsX = int(msg.x)
    gpsY = int(msg.y)

def MapCallback(msg):
    print "map callback"
    global gotMap, occGrid

    occGrid = msg
    gotMap = True

def WaveFront(x,y,i):
    global gpsX, gpsY, occGrid, pathFound, mapArr

    xindex = int((x-occGrid.info.origin.position.x)*(1.0/occGrid.info.resolution))
    yindex = int((y-occGrid.info.origin.position.y)*(1.0/occGrid.info.resolution))
    dataindex = yindex*occGrid.info.width + xindex

    if x == gpsX and y == gpsY:
        SteepestDescent(i)
        pathFound = True
    elif occGrid.data[dataindex] == 0 and mapArr[x][y] == i and not pathFound:
        print "recursing"
        mapArr[x+1][y]   = i+1
        mapArr[x+1][y+1] = i+1
        mapArr[x][y+1]   = i+1
        mapArr[x-1][y+1] = i+1
        mapArr[x-1][y]   = i+1
        mapArr[x-1][y-1] = i+1
        mapArr[x][y-1]   = i+1
        mapArr[x+1][y-1] = i+1

        WaveFront(x+1,y, i+1)   
        WaveFront(x+1,y+1, i+1) 
        WaveFront(x,y+1, i+1)   
        WaveFront(x-1,y+1, i+1) 
        WaveFront(x-1,y, i+1)   
        WaveFront(x-1,y-1, i+1) 
        WaveFront(x,y-1, i+1)   
        WaveFront(x+1,y-1, i+1)

def FindNext(x,y,i):

    global mapArr

    print type(x)

    if mapArr[x+1][y] == i-1:
        return (x+1,y)
    elif mapArr[x+1][y+1] == i-1:
        return (x+1,y+1)
    elif mapArr[x][y+1]    == i-1:
        return (x,y+1)
    elif mapArr[x-1][y+1]  == i-1:
        return (x-1, y+1)
    elif mapArr[x-1][y]   == i-1: 
        return (x-1,y)
    elif mapArr[x-1][y-1]  == i-1:
        return (x-1,y-1)
    elif mapArr[x][y-1]   == i-1:
        return (x,y-1)
    elif mapArr[x+1][y-1]  == i-1:
        return (x+1,y-1)


def SteepestDescent(iterations):
    print "found path"
    global gpsX, gpsY

    finalPath = Path()
    finalPath.header.frame_id = "map"

    x = gpsX
    y = gpsY

    print iterations
    for i in range(iterations, 0, -1):
        
        x, y = FindNext(x,y,i)

        p = PoseStamped()
        p.pose.position.x = x
        p.pose.position.y = y
        finalPath.poses.append(p)

    pub.publish(msg)
    rate.sleep()
    pathFound = False


gpsX = 0
gpsY = 0

goalX = 7
goalY = 2

mapWidth = 200
mapHeight = 200
mapRes = 0.1
occGrid = OccupancyGrid()
pathFound = False
gotMap = False

mapArr = [[0 for x in range(200)] for y in range(200)]
mapArr[goalX][goalY] = 1

rospy.init_node('planner')

pub = rospy.Publisher("path", Path, queue_size=10)

rate = rospy.Rate(10)

rospy.Subscriber("gps", Pose2D, GPSCallback)
rospy.Subscriber("map", OccupancyGrid, MapCallback)

while not rospy.is_shutdown():
    if gotMap:
        WaveFront(goalX, goalY, 1)
