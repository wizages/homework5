import rospy
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import Pose2D, PoseStamped
import math

def GPSCallback(msg):
    global gpsX, gpsY
    gpsX = (int(msg.x)*10)+99
    gpsY = (int(msg.y)*10)+99

def MapCallback(msg):
    print "map callback"
    global gotMap, occGrid

    occGrid = msg
    gotMap = True

def GetFillNeighborPoints(y,x):
    global mapArr, occGrid

    neighbors = []

    if y+1 <= 199:
        index = y*occGrid.info.width + x
        if mapArr[y+1][x] ==0 and occGrid.data[index] == 0:
            neighbors.append([y+1,x])


    if y-1 >=0 :
        index = y*occGrid.info.width + x
        if mapArr[y-1][x] ==0 and occGrid.data[index] == 0:
            neighbors.append([y-1,x])


    if x+1 <= 199:
        index = y*occGrid.info.width + x
        if mapArr[y][x+1] ==0 and occGrid.data[index] == 0:
            neighbors.append([y,x+1])


    if x-1 >= 0:
        index = y*occGrid.info.width + x
        if mapArr[y][x-1] ==0 and occGrid.data[index] == 0:
            neighbors.append([y,x-1])


    return neighbors

def GetDFSNeighborPoints(y,x):
    global mapArr, occGrid

    neighbors = []

    if y+1 <= 199:
       neighbors.append([y+1,x])


    if y-1 >=0 :
        neighbors.append([y-1,x])


    if x+1 <= 199:
        neighbors.append([y,x+1])


    if x-1 >= 0:
        neighbors.append([y,x-1])


    return neighbors

def WaveFront(y,x):
    global gpsX, gpsY, occGrid, pathFound, mapArr



    xindex = x#int(((x-99)/10-occGrid.info.origin.position.x)*(1.0/occGrid.info.resolution))
    yindex = y#int(((y-99)/10-occGrid.info.origin.position.y)*(1.0/occGrid.info.resolution))
    dataindex = yindex*occGrid.info.width + xindex

    i = 1
    nextPoints = []
    allPoints = [[y,x]]

    while mapArr[gpsY][gpsX] == 0:

        for point in allPoints:
            mapArr[point[0]][point[1]] = i

        i = i + 1
        print i


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

def FindNext(y,x,i):

    global mapArr

    print i
    if mapArr[y+1][x] == i-1:
        return (y+1,x)
    elif mapArr[y+1][x+1] == i-1:
        return (y+1,x+1)
    elif mapArr[y][x+1]    == i-1:
        return (y,x+1)
    elif mapArr[y-1][x+1]  == i-1:
        return (y-1, x+1)
    elif mapArr[y-1][x]   == i-1: 
        return (y-1,x)
    elif mapArr[y-1][x-1]  == i-1:
        return (x-1,x-1)
    elif mapArr[y][x-1]   == i-1:
        return (y,x-1)
    elif mapArr[y+1][x-1]  == i-1:
        return (y+1,x-1)


def DFS(coord,i, path=[]):
    global mapArr, dfsDone, goalY, goalX, PATHTOGOAL
    finalpath = []
    print i, coord[0], coord[1]
    print mapArr[coord[0]][coord[1]]
    if mapArr[coord[0]][coord[1]] != i:
        return path
    if coord[0] == goalY and coord[1] == goalX:
        dfsDone = True
        PATHTOGOAL = path

    path.append([coord[0],coord[1]])

    neighbors = GetDFSNeighborPoints(coord[0],coord[1])
    print neighbors

    if not dfsDone:
        for point in neighbors:
            finalpath = DFS(point,i-1, path)

    return finalpath

def SteepestDescent(iterations):
    print "found path"
    global gpsX, gpsY

    finalPath = Path()
    finalPath.header.frame_id = "map"

    x = gpsX
    y = gpsY
    
    DFS([y,x],iterations-1)
    print PATHTOGOAL

    for point in PATHTOGOAL:
        print point
        p = PoseStamped()
        p.pose.position.x = (point[1]-99)/10
        p.pose.position.y = (point[0]-99)/10
        finalPath.poses.append(p)


    '''for i in range(iterations, 0, -1):
                    
                    y,x = FindNext(x,y,i)
            
                    p = PoseStamped()
                    p.pose.position.x = (x-99)/10
                    p.pose.position.y = (y-99)/10
                    finalPath.poses.append(p)'''

    pub.publish(finalPath)
    rate.sleep()
    pathFound = False


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
dfsDone = False
PATHTOGOAL = []

mapArr = [[0 for x in range(200)] for y in range(200)]
mapArr[goalX][goalY] = 1

rospy.init_node('planner')

pub = rospy.Publisher("path", Path, queue_size=10)

rate = rospy.Rate(10)

rospy.Subscriber("gps", Pose2D, GPSCallback)
rospy.Subscriber("map", OccupancyGrid, MapCallback)

while not rospy.is_shutdown():
    if gotMap:
        iterations = WaveFront(goalY, goalX)
        SteepestDescent(iterations)
