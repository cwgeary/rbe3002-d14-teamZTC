#!/usr/bin/env python
import rospy, tf, math, numpy, copy
from nav_msgs.msg import OccupancyGrid #format for reading the map. Data is stored by row.
from nav_msgs.msg import GridCells #format for publishing to rviz display
from geometry_msgs.msg import PoseWithCovarianceStamped #format for reading the start and goal points
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Point
from std_msgs.msg import Header
from lab5Helpers import *

def readMap(msg):
    global mapInfo #map information such as width and hight, and cell sizes.
    global mapData #the cells of the map, with 100 = impassable and 0 = empty, -1 = unexplored. 
    global pub_map

    #print "resizing new map"
    resizedMap = mapResize(0.1, msg.info, msg.data)
    
    mapInfo = resizedMap.info
    mapData = resizedMap.data

    #print "expanding new map."
    mapData = obstacleExpansion(1, mapInfo, mapData)
    resizedMap.data = mapData

    print "publishing new map"
    pub_map.publish(resizedMap)

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
def expandPoint(radius, node, mapInfo):
    
    checkList = set()

    for i in range(radius*2+1):
        for j in range(radius*2+1):
            newNode = (node[0]-radius + i, node[1] - radius + j)
            if newNode[0] >= 0 and newNode[1] >= 0 and newNode[0] <= mapInfo.width and newNode[1] <= mapInfo.height:
                #temporarily store newNode to a variable (to prevent accidental loss of data if logic flaw exists)
                n = newNode
            elif newNode[0] <= 100 and newNode[0] >= radius and newNode[0] <= mapInfo.width and newNode[1] <= mapInfo.height:
                #...then set the adjacent node to the value of the current node, minus the given radius.
                newNode[1] = (newNode[0] - radius)
                #this prevents a node from being accidently converted to a unknown (-1) or an invalid value
                if newNode[1] < 0:
                    newNode[1] = 0
                    #and add the node to checkList
                checkList.add(newNode)
        else:
            checkList.add(n)
            #but otherwise just add 'n' to the checkList

    return checkList
    #return 'dat checkList, whatever happens

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

if __name__ == '__main__':
    rospy.init_node('map_processing_node')
    global mapInfo, mapData
    global odom_list
    global pub_map

    pub_map = rospy.Publisher('/newMap', OccupancyGrid)
    map_sum = rospy.Subscriber('map',OccupancyGrid, readMap, queue_size=1)

    odom_list = tf.TransformListener()

    rospy.sleep(rospy.Duration(1, 0))

    print "Map processing node started"

    rospy.spin()
