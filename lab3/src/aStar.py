
import rospy, tf, math
from lab3Helpers import *
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
		if mapData[gridToIndex(x, mapInfo, mapData)] is 0:
			points.append(x)
	return points

def sLH(node, goal):
	return math.sqrt(math.pow(goal[0]-node[0],2) + math.pow(goal[1]-node[1],2))

def costFunction(node, nextNode):
	return math.sqrt(math.pow(node[0]-nextNode[0],2) + math.pow(node[1]-nextNode[1],2))

def aStar(start, goal, mapInfo, mapData, pub_frontier, pub_expanded):
	
	parent = {} #dictanary of node_came_frome keyed on node
	expanded = [] #list of node
	frontier = [(start, sLH(start, goal))] #list of tuples (node cost) sorted by f(x) cost.
	costToNode = {start:0} #dictanary of g(x) cost, keyed on node
	goalInRange = False
	startInRange = False

	if(goal[0] >= 0 and goal[1] >= 0):
		goalInRange = True
	if(start[0] >= 0 and start[1] >= 0):
		startInRange = True

	while len(frontier) is not 0 and startInRange and goalInRange:

		#ser the current goal to the lowest frontier value
		frontier.sort(key=lambda fnode: fnode[1]) #sort the frontier, and return the lowest cost node
		current = frontier.pop(0)

		#publishGridList([node[0] for node in frontier] , mapInfo, pub_frontier)
		#publishGridList(expanded, mapInfo, pub_expanded)

		#if the current node is the goal, we found it!
		if current[0] == goal:
			return aStarPath(parent, start, goal)

		#remove the current from the frontier, and add it to the expanded list.
		expanded.append(current[0])

		#add all of the nodes that are next to the current node to the frontier
		for x in getBudds(current[0], mapInfo, mapData):
			#check to see if the buddy is in the expanded list already
			if x not in expanded:
				
				#compute the tenitive cost by adding the cost the the current + the cost to get to x from current
				tgx = costToNode[current[0]] + costFunction(x, current[0])

				frontierNodes = [node[0] for node in frontier]

				#if the node is not in the frontier, or if it has a higher cost somewhere else in the frontier
				if x not in frontierNodes or tgx < costToNode[x]:
					parent[x] = current[0]#set the parent of the node x to the current node
					costToNode[x] = costToNode[current[0]] + costFunction(x, current[0])
					if x not in frontierNodes:
						frontier.append((x, costToNode[x] + sLH(x ,goal)))

	print "no path found"
	return [[],[]]

#given a dictanary of the parents, and a goal, return the path to get to the goal.
def aStarPath(parent, start, goal):
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
	return [path, waypoints]


#if the last x direction does not match the new x direction...
#or if the new y does not match the old y....
def changedDirection(eldest, current, last):
	if not (eldest[0]-current[0] == current[0]-last[0] ) or not (eldest[1]-current[1] == current[1]-last[1]): 
		return True
	return False

				


	

