
import rospy, tf, math, numpy, copy
from nav_msgs.msg import OccupancyGrid #format for reading the map. Data is stored by row.
from nav_msgs.msg import GridCells #format for publishing to rviz display
from geometry_msgs.msg import PoseWithCovarianceStamped #format for reading the start and goal points
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Point
from std_msgs.msg import Header


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
def expandPoint(radius, node, mapInfo):
    
    checkList = set()

    for i in range(radius*2+1):
        for j in range(radius*2+1):
            newNode = (node[0]-radius + i, node[1] - radius + j)
            if newNode[0] >= 0 and newNode[1] >= 0 and newNode[0] <= mapInfo.width and newNode[1] <= mapInfo.height:
                checkList.add(newNode)
    return checkList

#returns a map that has been expanded by one cell.
def obstacleExpansion(radius, mapInfo, mapData):
    expanded = set()
    #dictanary of int keyed on node.
    newMap = {}
    newMapData = {}
    newMapCells = []

    #print "creating the new map"

    for i in range(len(mapData)):
        newMap[indexToGrid(i, mapInfo)] = mapData[i]

    #print "expanding"
    for node in newMap:
        if newMap[node] == 100:
            expanded.update(expandPoint(radius, node, mapInfo))

    #print "length of expanded"
    print len(expanded)

    for node in expanded:
        newMap[node] = 100

    #print "generating new map data"
    for node in newMap:
        newMapData[gridToIndex(node, mapInfo)] = newMap[node]


    for i in range(len(newMapData)):
        newMapCells.append(newMapData[i])

    return newMapCells

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

#takes a map object and a new resolution and returns the Occupancy grid at the new resolution.
def mapResize(newRes, mapInfo, mapData):
    oldRes = mapInfo.resolution
    oldw = mapInfo.width
    oldh = mapInfo.height

    oldMapInfo = copy.deepcopy(mapInfo)

    nMapData = []
    nMapDataD = {}

    #change the size of the new map. The offsets of the new map are already copied from the old map.
    mapInfo.resolution = newRes
    mapInfo.width = int(round( (oldw*(oldRes/newRes)) ))
    mapInfo.height = int(round( (oldh*(oldRes/newRes)) ))

    #print "old map dimensions"
    #print oldMapInfo.width
    #print oldMapInfo.height

    #print "new map dimensions w x h"
    #print mapInfo.width
    #print mapInfo.height

    #populate the new map at the defined resolution with all cells at -1
    #print "generating the new map"
    
    for x in range(mapInfo.width * mapInfo.height):
        nMapDataD[x] = 0

    #print "summing new blocks"
    for i in range(len(mapData)):
        gp = gridToGlobal(indexToGrid(i, oldMapInfo), oldMapInfo)
        nIndex = gridToIndex(globalToGrid(gp, mapInfo), mapInfo)
        
        if nIndex >= (mapInfo.width * mapInfo.height -1):
            nIndex = mapInfo.width * mapInfo.height-1
        nMapDataD[nIndex] = (nMapDataD[nIndex] + mapData[i])

    for i in nMapDataD:
        value = nMapDataD[i] / ((newRes*newRes) - (oldRes * oldRes))
        if value > 5:
            value = 100
        elif value < 0:
            value = -1
        else:
            value = 0
        nMapData.append(numpy.int8(value))

    newMapOC = OccupancyGrid()
    newMapOC.info = mapInfo
    newMapOC.data = tuple(nMapData)

    return newMapOC









