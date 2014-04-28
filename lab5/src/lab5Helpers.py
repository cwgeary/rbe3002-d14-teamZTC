
import rospy, tf, math, numpy, copy
from nav_msgs.msg import OccupancyGrid #format for reading the map. Data is stored by row.
from nav_msgs.msg import GridCells #format for publishing to rviz display
from geometry_msgs.msg import PoseWithCovarianceStamped #format for reading the start and goal points
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Point
from std_msgs.msg import Header

#given a gridPoint, return a list of points that are reachable.
def getBudds(p, mapInfo, mapData):
    #define a set of all the the points that are adjecent to the current point
    possiblePoints = [ (p[0]-1,p[1]+1), (p[0],p[1]+1), (p[0]+1,p[1]+1), (p[0]+1,p[1]), (p[0]+1,p[1]-1), (p[0],p[1]-1), (p[0]-1,p[1]-1), (p[0]-1,p[1]) ]

    points = []
    for x in possiblePoints:
        if x[0] >= 0 and x[0] <= mapInfo.width-1 and x[1] >= 0 and x[1] <= mapInfo.height-1:
            if mapData[gridToIndex(x, mapInfo)] <= 1:
                points.append(x)
    return points

#takes a gridPoint and converts it into a global point for display in rviz.
def gridToGlobal(gridPoint, mapInfo):
    
    point = Point()
    point.x = gridPoint[0] * mapInfo.resolution + mapInfo.origin.position.x + mapInfo.resolution/2
    point.y = gridPoint[1] * mapInfo.resolution + mapInfo.origin.position.y + mapInfo.resolution/2
    point.z = 0

    return point

#takes a point in the global frame and returns the point value for the given cell in the map frame as a tuple.
#note that zero is set as the LOWER left corner, with +x to the right and +y down.
def globalToGrid(point, mapInfo):
    
    offsetX = mapInfo.origin.position.x
    offsetY = mapInfo.origin.position.y

    gridPoint = (int((point.x - offsetX)/ mapInfo.resolution), int((point.y - offsetY)/ mapInfo.resolution))

    return gridPoint

def indexToGrid(index, mapInfo):
    gridPoint = (index % mapInfo.width , index / mapInfo.width)
    return gridPoint


#convert the grid point into a map index value, and outputs it.
def gridToIndex(gridPoint, mapInfo):

    return gridPoint[0] + gridPoint[1] * (mapInfo.width)

def pubMap(publisher, mapInfo, mapData):
    cells = []
    for i in range(mapInfo.width * mapInfo.height):
        if mapData[i] == 100:
            gridPoint = indexToGrid(i, mapInfo)
            gp = gridToGlobal(gridPoint, mapInfo)
            cells.append(gp)

    gridCells = GridCells()
    gridCells.header = Header()
    gridCells.header.frame_id = "map"
    gridCells.cell_width = mapInfo.resolution
    gridCells.cell_height = mapInfo.resolution
    gridCells.cells = cells
    publisher.publish(gridCells)

def publishGridList(list_of_cells, mapInfo, pub):
    cells = []
    for x in list_of_cells:
        cells.append(gridToGlobal(x, mapInfo))
    
    gridCells = GridCells()
    gridCells.header = Header()
    gridCells.header.frame_id = "map"
    gridCells.cell_width = mapInfo.resolution
    gridCells.cell_height = mapInfo.resolution
    gridCells.cells = cells

    pub.publish(gridCells)

def getAllAdj(depth, indexValue, mapInfo, mapData):
    #cover the majority of the test cases with the first if statement.
    toAdd = []

    w = mapInfo.width
    if indexValue > w and indexValue < w*(mapInfo.height-1) and indexValue % w > 1:
        toAdd = [indexValue-w-1, indexValue-w, indexValue-w+1,indexValue-1,indexValue+1,indexValue+w-1,indexValue+w,indexValue+1]
    return toAdd

def ecludianDist(node, nextNode):
    return math.sqrt(math.pow(node[0]-nextNode[0],2) + math.pow(node[1]-nextNode[1],2))


#this assumes that the node is more than radius away from any edge of the map.
#returns the list of the points that are around the given point.
#def expandPoint(radius, node, mapInfo):
#    
#    checkList = set()
#
#    for i in range(radius*2+1):
#        for j in range(radius*2+1):
#            newNode = (node[0]-radius + i, node[1] - radius + j)
#            if newNode[0] >= 0 and newNode[1] >= 0 and newNode[0] <= mapInfo.width and newNode[1] <= mapInfo.height:
#                checkList.add(newNode)
#    return checkList

#this assumes that the node is more than radius away from any edge of the map.
#returns the list of the points that are around the given point.
#The value of each free node (0) adjacent to the current non-zero node is incremented to the value of the non-zero node minus the given radius, 
#effectively creating a "fuzzy" boundry between free space and obstacles
#Note that this is NOT Tested

#if the last x direction does not match the new x direction...
#or if the new y does not match the old y....
def changedDirection(eldest, current, last):
    if not (eldest[0]-current[0] == current[0]-last[0] ) or not (eldest[1]-current[1] == current[1]-last[1]): 
        return True
    return False

#return the next node in the direction of travel. If the next node is off of the map it will return the current node.
def nextInDirection(parent, node, mapInfo, mapData):
    dx = node[0] - parent[0]
    dy = node[1] - parent[1]
    nextNode = (node[0]+dx, node[1]+dy)
    
    if nextNode[0] <= 0 or nextNode[0] >= mapInfo.width-1:
        return node
    
    if nextNode[1] <= 0 or nextNode[1] >= mapInfo.height-1:
        return node
    
    if mapData[gridToIndex(nextNode, mapInfo)] == 100:
        return node

    return nextNode


#takes two nodes, and returns an int between 1 and 8 to indecate the direction of travel.
#1 2 3
#8 0 4
#7 6 5
#I feel like there should be a better way to do this, but I couldn't think of one off of the top of my head.
def getDirection(eldest, current):
    x = current[0]
    y = current[1]
    xd = eldest[0]
    yd = eldest[1]
    if(x < xd and y>yd):
        return 1
    if(x==xd and y>yd):
        return 2
    if(x>xd and y>yd):
        return 3
    if(x>xd and y == yd):
        return 4
    if(x>xd and y < yd):
        return 5
    if(x==xd and y<yd):
        return 6
    if(x<xd and y<yd):
        return 7
    if(x<xd and y==yd):
        return 8
    return 0