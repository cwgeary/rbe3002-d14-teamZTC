#!/usr/bin/env python

import rospy, tf, math
from heapq import *
from lab5Helpers import *
from geometry_msgs.msg import PoseWithCovarianceStamped #format for reading the start and goal points
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Point
from geometry_msgs.msg import Point
from nav_msgs.msg import OccupancyGrid #format for reading the map. Data is stored by row.
from std_msgs.msg import Header
from lab5.srv import *


def readMap(msg):
	#mapData = msg.data
	#mapInfo = msg.info
	print "Map Received"

	global mapInfo, mapData, goalPoint, waypoints

	mapInfo = msg.info
	mapData = msg.data

	#if(len(mapData) != mapInfo.width * mapInfo.height):
	#    print "map size does not match data length."
	#    print len(mapData)

	(tran, rot) = odom_list.lookupTransform('map','base_footprint', rospy.Time(0))
	startPoint = Point()
	startPoint.x = tran[0]
	startPoint.y = tran[1]

	paths = aStar(globalToGrid(startPoint, mapInfo), globalToGrid(goalPoint ,mapInfo))
	waypoints = paths[1]



def expandCurrent(goal, node, mapInfo, mapData):
	buddies = getBudds(node, mapInfo, mapData)
	children = []

	for bud in buddies:
		if bud == goal:
			return [bud]

		if getDirection(node, bud) % 2 == 0:
			children = children + linearJump(goal, node, bud, mapInfo, mapData)
		else:
			children = children + diagJump(goal, node, bud, mapInfo, mapData)

	return children


def diagJump(goal, parent, node, mapInfo, mapData):
	nodeList = []
	nextNode = (-1,-1)
	while 1:
		
		sideNode = linearJump(goal, (parent[0], parent[1]+(node[1]-parent[1])) , node, mapInfo, mapData)
		verticalNode = linearJump(goal, (parent[0]+(node[0]-parent[0]), parent[1]) , node, mapInfo, mapData)
		
		if hasForcedNeighbor(parent, node, mapInfo, mapData) or len(sideNode+verticalNode) > 0 or nextInDirection(parent, node, mapInfo, mapData) == goal:
			return [node]

		if nextInDirection(parent, node, mapInfo, mapData) == node:
			return []
		
		nextNode = nextInDirection(parent, node, mapInfo, mapData)
		parent = node
		node = nextNode

	return []

#function for the jump part of the JPS
#returns either a list with one node, or a empty list if there are no valid jumps.
def linearJump(goal, parent, node, mapInfo, mapData):	
	nextNode = (-1,-1)
	#if the next in the direction is the edge of the map or a wall return a empty list.
	while not nextInDirection(parent, node, mapInfo, mapData) == node:
		if hasForcedNeighbor(parent, node, mapInfo, mapData) or nextInDirection(parent, node, mapInfo, mapData) == goal:
			return [node]

		nextNode = nextInDirection(parent, node, mapInfo, mapData)
		parent = node
		node = nextNode
	return []

#this function is dumb.
def hasForcedNeighbor(parent, node, mapInfo, mapData):
	#check to see if the next node is a wall.
	if nextInDirection(parent, node, mapInfo, mapData) == node:
		return False


	direction = getDirection(parent, node)
	tnode1 = (-1,-1)
	tnode2 = (-1,-1)
	n1 = (node[0]-1, node[1]+1)
	n2 = (node[0], node[1]+1) 
	n3 = (node[0]+1, node[1]+1)
	n4 = (node[0]+1, node[1])
	n5 = (node[0]+1, node[1]-1)
	n6 = (node[0], node[1]-1)
	n7 = (node[0]-1, node[1]-1)
	n8 = (node[0]-1, node[1])

	if direction == 2:
		if mapData[gridToIndex(n8, mapInfo)] == 100 and mapData[gridToIndex(n1,mapInfo)] == 0: 
			return True
		if mapData[gridToIndex(n4, mapInfo)] == 100 and mapData[gridToIndex(n3,mapInfo)] == 0: 
			return True
	elif direction == 6:
		if mapData[gridToIndex(n8, mapInfo)] == 100 and mapData[gridToIndex(n7,mapInfo)] == 0: 
			return True
		if mapData[gridToIndex(n4, mapInfo)] == 100 and mapData[gridToIndex(n5,mapInfo)] == 0: 
			return True
	elif direction == 4:
		if mapData[gridToIndex(n2, mapInfo)] == 100 and mapData[gridToIndex(n3,mapInfo)] == 0: 
			return True
		if mapData[gridToIndex(n6, mapInfo)] == 100 and mapData[gridToIndex(n5,mapInfo)] == 0: 
			return True
	elif direction == 8:
		if mapData[gridToIndex(n2, mapInfo)] == 100 and mapData[gridToIndex(n1,mapInfo)] == 0: 
			return True
		if mapData[gridToIndex(n6, mapInfo)] == 100 and mapData[gridToIndex(n7,mapInfo)] == 0: 
			return True
	elif direction == 1:
		if mapData[gridToIndex(n4, mapInfo)] == 100 and mapData[gridToIndex(n3,mapInfo)] == 0: 
			return True
		if mapData[gridToIndex(n6, mapInfo)] == 100 and mapData[gridToIndex(n7,mapInfo)] == 0: 
			return True
	elif direction == 3:
		if mapData[gridToIndex(n6, mapInfo)] == 100 and mapData[gridToIndex(n5,mapInfo)] == 0:
			return True
		if mapData[gridToIndex(n8, mapInfo)] == 100 and mapData[gridToIndex(n1,mapInfo)] == 0: 
			return True
	elif direction == 5:
		if mapData[gridToIndex(n2, mapInfo)] == 100 and mapData[gridToIndex(n3,mapInfo)] == 0: 
			return True
		if mapData[gridToIndex(n8, mapInfo)] == 100 and mapData[gridToIndex(n7,mapInfo)] == 0: 
			return True
	elif direction == 7:
		if mapData[gridToIndex(n2, mapInfo)] == 100 and mapData[gridToIndex(n1,mapInfo)] == 0: 
			return True
		if mapData[gridToIndex(n4, mapInfo)] == 100 and mapData[gridToIndex(n5,mapInfo)] == 0: 
			return True
	#print "no node found"
	return False


def sLH(node, goal):
	return ecludianDist(node, goal)

def costFunction(node, nextNode):
	return ecludianDist(node, nextNode)

def aStar(start, goal):
	global mapData, mapInfo
	global pub_frontier, pub_expanded
	
	parent = {} #dictanary of node_came_from keyed on node
	expanded = [] #list of node
	frontier = [] #list of tuples (node cost) sorted by f(x) cost.
	heappush(frontier, (sLH(start, goal), start))
	costToNode = {start:0} #dictanary of g(x) cost, keyed on node
	goalInRange = False
	startInRange = False

	if(goal[0] >= 0 and goal[1] >= 0):
		goalInRange = True
	if(start[0] >= 0 and start[1] >= 0):
		startInRange = True

	while len(frontier) is not 0 and startInRange and goalInRange:
		#ser the current goal to the lowest frontier value, and remove it from the frontier
		current = heappop(frontier)

		publishGridList([node[1] for node in frontier] , mapInfo, pub_frontier)
		publishGridList(expanded, mapInfo, pub_expanded)

		#if the current node is the goal, we found it!
		if current[1] == goal:
			print "path found"
			return aStarPath(parent, start, goal)

		# add current to the expanded list.
		expanded.append(current[1])

		#add all of the nodes that are next to the current node to the frontier
		for x in expandCurrent(goal, current[1], mapInfo, mapData):
			#check to see if the buddy is in the expanded list already
			if x not in expanded:
				#compute the tenitive cost by adding the cost the the current + the cost to get to x from current
				tgx = costToNode[current[1]] + costFunction(x, current[1])

				frontierNodes = [node[1] for node in frontier]

				#if the node is not in the frontier, or if it has a higher cost somewhere else in the frontier
				if x not in frontierNodes or tgx < costToNode[x]:
					parent[x] = current[1] #set the parent of the node x to the current node
					costToNode[x] = costToNode[current[1]] + costFunction(x, current[1])
					if x not in frontierNodes:
						heappush(frontier, (costToNode[x] + sLH(x ,goal), x))

	print "no path found"
	return [[],[]]

#given a dictanary of the parents, and a goal, return the path to get to the goal.
def aStarPath(parent, start, goal):
	global waypoints

	path = []
	waypoints = []
	current = goal
	last = goal
	while current is not start:
		path.append(current)
		last = current
		current = parent[current]

		if current is not start:
			eldest = parent[current]

			if changedDirection(eldest, current, last):

				waypoints.append(current) #then we have changed direction!

		del parent[last]
	waypoints.append(start)

	publishGridList(waypoints, mapInfo, pub_path)

	return [path, waypoints]

def handle_aStar_request(req):
	global odom_list, waypoints, goalPoint

	goalPoint = Point()
	goalPoint.x = req.x
	goalPoint.y = req.y

	xList = []
	yList = []

	for i in range(len(waypoints)):
		p = gridToGlobal(waypoints[i], mapInfo)
		xList.append(p.x)
		yList.append(p.y)
	return AStarResponse(xList,yList)

def aStar_server():
	global odom_list
	rospy.init_node('aStar_server')
	s = rospy.Service('aStar', AStar, handle_aStar_request)
	print "Awaiting Map"
	map_sum = rospy.Subscriber('/newMap', OccupancyGrid, readMap, queue_size=1)
	odom_list = tf.TransformListener()

	rospy.spin()

if __name__ == "__main__":
	global pub_expanded, pub_frontier
	
	pub_frontier = rospy.Publisher('/frontier', GridCells)
	pub_expanded = rospy.Publisher('/expanded', GridCells)
	pub_path = rospy.Publisher('/path', GridCells)



	aStar_server()

