#!/usr/bin/env python

#python imports
import rospy, tf, math
from lab5Helpers import *
from aStar import getBudds

#booliean used in frontierId. returns true if at least one of the neighboring cells is open.
def nearKnown(node, mapInfo, mapData):
	neighbors = getBudds(node, mapInfo, mapData)
	for n in neighbors:
		if mapData[gridToIndex(n, mapInfo)] <= 20 and mapData[gridToIndex(n, mapInfo)] >= 0:
			return True
	return False

#Frontier Identification
#takes a map, and returns a list of nodes that are frontiers.
def frontierId(mapInfo, mapData):
	frontier = []
	for i in range(mapInfo.width*mapInfo.height):
		if mapData[i] == -1 and nearKnown(indexToGrid(i, mapInfo), mapInfo, mapData):
			frontier.append(indexToGrid(i, mapInfo))
	return frontier

#takes a contious segment of the frontier, and returns the centroid.
def centroid(frontierSet, mapInfo):
	length = len(frontierSet)
	xc = 0
	yc = 0
	for n in frontierSet:
		xc = n[0] + xc
		yc = n[1] + yc
	xc = xc / length
	yc = yc / length
	return gridToGlobal((xc, yc), mapInfo)

#takes a map and a current location and returns a waypoint that is a weighted balance between 
#the size of the frontier segment and the distance from the current location.
def frontierWaypoints(globalPoint, mapInfo, mapData):
	# a dict of id numbers keyed on nodes. This is how we will check the id of the neighboring nodes.
	frontier = {}
	frontierList = frontierId(mapInfo, mapData)		#a list of nodes 
	frontierSets = {}								#a dict of lists of node keyed on id values
	buds = []
	centroids = []

	print "frontierList is " + str(len(frontierList)) + " long."

	frontier = {key: -1 for key in frontierList}

	idnum = 0
	for n in frontierList:
		#assign the node to a unique id number. Then increment the id counter.
		frontier[n] = idnum
		

		#get the neighbors (if any) that are also in the frontier
		buds = list(set(getBudds(n, mapInfo, mapData)).intersection(frontierList))

		for bud in buds:
			#check to see if the buddy value has been set, and check to see if the current value has not been set.
			#this should be the case for the first pass through the loop, and is all we need in most cases. the following
			#lines are mostly for conflict resolution.
			if frontier[bud] != -1 and frontier[n] == idnum:
				frontier[n] = frontier[bud]

			#This is to handle the case of the next node has been set, but it is different fron
			#the current node and from the last node. In this case we would like to chance the value of
			#all of the nodes that use this key to whichever key is the lowest of the two valid keys.
			elif frontier[n] != idnum and frontier[n] != frontier[bud] and frontier[bud] != -1:
				#pick whichever id value is lower, and set all of the conflicting nodes to that value as well.
				if frontier[n] < frontier[bud]:
					#get all of the nodes that key for the higher value
					remapping = [node for node, idval in frontier.items() if idval == frontier[n]]
					for m in remapping:
						frontier[m] = frontier[n]

				elif frontier[bud] < frontier[n]:
					#all of the nodes that are keyed to frontier[n]
					remapping = [node for node, idval in frontier.items() if idval == frontier[bud]]
					for m in remapping:
						frontier[m] = frontier[bud]
		idnum = idnum + 1

	#at this point each of the nodes should have been assigned an id number that will tell us which grouping it
	#belongs to. We will now use the length as the "weight" of the grouping, and the centroid to calculate the 
	#distance and set the waypoint.

	#print "the values of the frontier"
	f = frontier.values()
	f.sort()
	#print f


	#process the frontier dict and use it to fill a dict of lists of nodes based on the ids of the nodes.
	for node in frontier:
		#print "for node: " + str(node)
		#print "there is key: " + str(frontier[node])

		if frontier[node] in frontierSets:
			templist = frontierSets[frontier[node]]
		else: templist = []

		templist.append(node)

		#print "templist"
		#print templist

		frontierSets[frontier[node]] = templist

	#now to calculate the weight and centroid of each section of the frontier.
	for idval in frontierSets:
		frontierSet = frontierSets[idval]

		cent = centroid(frontierSet, mapInfo)
		weight = len(frontierSet)
		distance = math.sqrt(math.pow(globalPoint.x-cent.x,2) + math.pow(globalPoint.y-cent.y,2))

		centroids.append( (cent, weight, distance) )

	print "centroids is " + str(len(centroids)) + " long."

	#sort the centroid list by weight and distance

	return centroids


def frontierIsDone(mapInfo, mapData):
	if len(frontierId(mapInfo, mapData)) == 0:
		return True
	else:
		return False


def readMap(msg):
    global mapInfo #map information such as width and hight, and cell sizes.
    global mapData #the cells of the map, with 100 = impassable and 0 = empty, -1 = unexplored. 
    global pub_exway
    global odom_list
    global startPoint
    global frontier, frontierPoints

    (tran, rot) = odom_list.lookupTransform('map','base_footprint', rospy.Time(0))

    startPoint = Point()
    startPoint.x = tran[0]
    startPoint.y = tran[1]

    mapInfo = msg.info
    mapData = msg.data

    frontier = frontierId(mapInfo, mapData)
    frontierPoints = frontierWaypoints(startPoint, mapInfo, mapData)

    print "centroids weights"
    print [n[1] for n in frontierPoints]
    print "centroids distances"
    print [n[2] for n in frontierPoints]

    publishGridList([globalToGrid(n[0],mapInfo) for n in frontierPoints], mapInfo, pub_exway)


# This is the program's main function
if __name__ == '__main__':
    # Change this node name to include your username
    rospy.init_node('Lab5_Exploration_node')

    global mapInfo, mapData, frontierPoints, frontier
    global pub_exway

    goal = (-1,-1)
    start = (-2,-2)
    frontier = [goal, start]

    #Set up the subscriptions to all of the nessaray data
    map_sum = rospy.Subscriber('newMap',OccupancyGrid, readMap, queue_size=1)
    pub_exway = rospy.Publisher('/explorationWaypoints', GridCells)

    #a publisher that sends the goal point that was calculated from the centroids.

    #this is here to get the current position of the robot
    odom_list = tf.TransformListener()

    # Use this command to make the program wait for some seconds
    rospy.sleep(rospy.Duration(1, 0))


    print "Starting exploration node"

    r = rospy.Rate(10)
    while not rospy.is_shutdown() and len(frontier) > 0:

        r.sleep()

    print "The map has been fully explored. Shutting down exploration node."