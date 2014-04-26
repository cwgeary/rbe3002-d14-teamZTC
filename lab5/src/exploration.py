#python imports
import rospy, tf, math
from lab5Helpers import *
from aStar import getBudds

#booliean used in frontierId. returns true if at least one of the neighboring cells is open.
def nearKnown(node, mapInfo, mapData):
	neighbors = getBudds(node, mapInfo, mapData)
	for n in neighbors:
		if mapData(gridToIndex(node, mapInfo)) <= 20 and mapData(gridToIndex(node, mapInfo)) >= 0:
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
	for n in frontierSet:
		xc = n[0] + xc
		yc = n[1] + yc
	xc = xc / length
	yc = yc / length
	return ((xc, yc), length)

#takes a map and a current location and returns a waypoint that is a weighted balance between 
#the size of the frontier segment and the distance from the current location.
def frontierWaypoints(globalPoint, mapInfo, mapData):
	# a dict of id numbers keyed on nodes. This is how we will check the id of the neighboring nodes.
	frontier = {}
	frontierList = frontierId(mapInfo, mapData)		#a list of nodes 
	frontierSets = {}								#a dict of lists keyed on id values
	buds = []
	centroids []

	frontier = {key: -1 for key in frontierList}

	idnum = 0
	for n in frontierList:
		#assign the node to a unique id number. Then increment the id counter.
		frontier[n] = idnum
		idnum = idnum + 1

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
			else if frontier[n] != idnum and frontier[n] != frontier[bud] and frontier[bud] != -1:
				#pick whichever id value is lower, and set all of the conflicting nodes to that value as well.
				if frontier[n] < frontier[bud]:
					#get all of the nodes that key for the higher value
					remapping = [node for node, idval in frontier.items() if idval == frontier[n]]
					for m in remapping:
						frontier[m] = frontier[n]

				else if frontier[bud] < frontier[n]:
					#all of the nodes that are keyed to frontier[n]
					remapping = [node for node, idval in frontier.items() if idval == frontier[bud]]
					for m in remapping[]:
						frontier[m] = frontier[bud]

	#at this point each of the nodes should have been assigned an id number that will tell us which grouping it
	#belongs to. We will now use the length as the "weight" of the grouping, and the centroid to calculate the 
	#distance and set the waypoint.

	for node in frontier:
		templist = frontierSets[frontier[node]]
		templist = templist.append(node)
		frontierSets[frontier[node]] = templist

	#now to calculate the weight and centroid of each section of the frontier.
	for idval in frontierSets:
		frontierSet = frontierSets[idval]
		centroids.append[centroid(frontierSet, mapInfo)]

	return centroids


def frontierIsDone(mapInfo, mapData):
	if len(frontierId(mapInfo, mapData)) == 0:
		return True
	else:
		return False