import rospy
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import Pose2D, PoseStamped
import math

def GPSCallback(msg):
    global gpsX, gpsY, needGPS
    if gpsX != int(msg.x*10)+99 and gpsY != int(msg.y*10)+99 and needGPS:
        gpsX = int(msg.x*10)+99
        gpsY = int(msg.y*10)+99

def MapCallback(msg):
    #print "map callback"
    global gotMap, occGrid

    occGrid = msg
    gotMap = True

def CopyOccGrid(occGrid):
    gridCopy = []
    for y in range(200):
        gridCopy.append([])
        for x in range(200):
            index = y * occGrid.info.width + x
            gridCopy[y].append(occGrid.data[index])

    return gridCopy

def GetFillNeighborPoints(y,x):
    global mapArr, gridCopy

    neighbors = []

    if y+1 <= 199:
        index = y*occGrid.info.width + x
        if mapArr[y+1][x] ==0 and gridCopy[y+1][x] == 0:
            neighbors.append([y+1,x])


    if y-1 >=0 :
        index = y*occGrid.info.width + x
        if mapArr[y-1][x] ==0 and gridCopy[y-1][x] == 0:
            neighbors.append([y-1,x])


    if x+1 <= 199:
        index = y*occGrid.info.width + x
        if mapArr[y][x+1] ==0 and gridCopy[y][x+1] == 0:
            neighbors.append([y,x+1])


    if x-1 >= 0:
        index = y*occGrid.info.width + x
        if mapArr[y][x-1] ==0 and gridCopy[y][x-1] == 0:
            neighbors.append([y,x-1])


    return neighbors

def GetDFSNeighborPoints(y,x):
    global gridCopy
    neighbors = []


    if x+1 <= 199:
        index = y*occGrid.info.width + x
        if gridCopy[y][x+1] == 0:
            neighbors.append([y,x+1])

    if y+1 <= 199:
        index = y*occGrid.info.width + x
        if gridCopy[y+1][x] == 0:
            neighbors.append([y+1,x])


    if y-1 >=0 :
        index = y*occGrid.info.width + x
        if gridCopy[y-1][x] == 0:
            neighbors.append([y-1,x])


    if x-1 >= 0:
        index = y*occGrid.info.width + x
        if gridCopy[y][x-1] == 0:
            neighbors.append([y,x-1])


    return neighbors

def WaveFront(y,x):
    global gpsX, gpsY, pathFound, mapArr

    i = 1
    nextPoints = []
    allPoints = [[y,x]]

    while mapArr[gpsY][gpsX] == 0:

        for point in allPoints:
            mapArr[point[0]][point[1]] = i

        i = i + 1

        for point in allPoints:
            nextPoints.append(GetFillNeighborPoints(point[0],point[1]))

        justPoints = []
        for pointset in nextPoints:
            for point in pointset:
                justPoints.append(point)

        uniquePoints = []
        [uniquePoints.append(item) for item in justPoints if item not in uniquePoints]

        allPoints = uniquePoints
        nextPoints = []

    return i


def DFS(coord,i, path=[]):
    global mapArr, dfsDone, goalY, goalX, PATHTOGOAL

    finalpath = []
    if mapArr[coord[0]][coord[1]] != i or dfsDone:
        return

    if coord[0] == goalY and coord[1] == goalX:
        dfsDone = True
        PATHTOGOAL = path

    path.append([coord[0],coord[1]])

    neighbors = GetDFSNeighborPoints(coord[0],coord[1])

    if not dfsDone:
        for point in neighbors:
            finalpath = DFS(point,i-1, path)


def SteepestDescent(iterations):
    global gpsX, gpsY, PATHTOGOAL
    print "found path"
    print gpsX, gpsY
    finalPath = Path()
    finalPath.header.frame_id = "map"

    x = gpsX
    y = gpsY
    
    DFS([y,x],iterations-1)
    #print PATHTOGOAL

    for point in PATHTOGOAL:
        #print point
        p = PoseStamped()
        p.pose.position.x = (point[1]-99)/10.0
        p.pose.position.y = (point[0]-99)/10.0
        finalPath.poses.append(p)

    pub.publish(finalPath)
    #rate.sleep()
    pathFound = False

def IgniteFire(y,x, fireList):

    if x+1 <= 199:
        fireList[y][x+1] = 100

    if y+1 <= 199:
        fireList[y+1][x] = 100

    if y-1 >=0 :
        fireList[y-1][x] = 100

    if x-1 >= 0:
        fireList[y][x-1] = 100


def BrushFire():
    global gridCopy

    fireList = []

    for y in range(200):
        fireList.append([])
        for x in range(200):
            fireList[y].append(gridCopy[y][x])
            


    for y in range(200):
        for x in range(200):
            if gridCopy[y][x] == 100:
                IgniteFire(y,x, fireList)

    for y in range(200):
        for x in range(200):
            if fireList[y][x] == 100:
                IgniteFire(y,x,gridCopy)

    #return fireList

gpsX = 0
gpsY = 0

goalX = (7*10)+100
goalY = (2*10)+100

mapWidth = 200
mapHeight = 200
mapRes = 0.1
occGrid = OccupancyGrid()
pathFound = False
gotMap = False
needGPS = True
dfsDone = False
PATHTOGOAL = []
gridCopy = []

mapArr = [[0 for x in range(200)] for y in range(200)]
mapArr[goalX][goalY] = 1

rospy.init_node('planner')

pub = rospy.Publisher("path", Path, queue_size=10)

rate = rospy.Rate(10)

rospy.Subscriber("gps", Pose2D, GPSCallback)
rospy.Subscriber("map", OccupancyGrid, MapCallback)

while not rospy.is_shutdown():
    if gotMap:
        needGPS = False
        gridCopy = CopyOccGrid(occGrid)
        BrushFire()
        iterations = WaveFront(goalY, goalX)
        SteepestDescent(iterations)
        needGPS = True
        #rate.sleep()
