#!/usr/bin/env python


import rospy, tf, math
from lab3Helpers import *
from aStar import *
from std_msgs.msg import Header
from nav_msgs.msg import OccupancyGrid #format for reading the map. Data is stored by row.
from nav_msgs.msg import GridCells #format for publishing to rviz display
from geometry_msgs.msg import PoseWithCovarianceStamped #format for reading the start and goal points
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Point


#this is strictly path finding code. As this node will be used as such in later labs
#This node automaticly subcribes to the map topic, and  publishis a service called getPath. getPath takes a 
#starting point and a goal point and then publishes the frontier, and the optimal path.

def readMap(msg):
    global mapInfo #map information such as width and hight, and cell sizes.
    global mapData #the cells of the map, with 100 = impassable and 0 = empty, -1 = unexplored. 

    mapInfo = msg.info
    mapData = msg.data
    
    if(len(mapData) != mapInfo.width * mapInfo.height):
        print "map size does not match data length."

def setStart(msg):
    global start, pub_start, mapInfo, mapData

    #define a point object that will represent our start point on the grid
    point = msg.pose.pose.position #this needs to be adjuseted depending on how the gridcells object is using the point object.

    #set the starting point for the search to the gridpoint defined by the user mouse click.
    start = globalToGrid(point, mapInfo)

    #convert the point to a grid position
    point.x = round(point.x/mapInfo.resolution) * mapInfo.resolution
    point.y = round(point.y/mapInfo.resolution) * mapInfo.resolution

    #define a new gridcells object 
    gridCells = GridCells()
    
    #start construction our message
    gridCells.header = msg.header
    gridCells.cell_width = mapInfo.resolution
    gridCells.cell_height = mapInfo.resolution
    cells = [point]
    gridCells.cells = cells

    pub_start.publish(gridCells)
    print "startpoint set"


def setGoal(msg):
    global goal, pub_goal
    point = msg.pose.position

    #set the goal point for the search to the gridpoint defined by the user mouse click.
    goal = globalToGrid(point, mapInfo)

    #point.x = round(point.x/mapInfo.resolution) * mapInfo.resolution
    #point.y = round(point.y/mapInfo.resolution) * mapInfo.resolution
    point = gridToGlobal(goal, mapInfo)

    #define a new gridcells object 
    gridCells = GridCells()
    
    #start construction our message
    gridCells.header = msg.header
    gridCells.cell_width = mapInfo.resolution
    gridCells.cell_height = mapInfo.resolution
    cells = [point]
    gridCells.cells = cells

    pub_goal.publish(gridCells)
    print "goal set"

# This is the program's main function
if __name__ == '__main__':
    # Change this node name to include your username
    rospy.init_node('Lab_3_node')

    global mapInfo, mapData
    global frontier, expanded, path, start, goal

    global pub_start, pub_goal, pub_frontier, pub_path, pub_expanded, pub_waypoints

    goal = (-1,-1)
    start = (-2,-2)

    #Set up the subscriptions to all of the nessaray data
    map_sum = rospy.Subscriber('map',OccupancyGrid, readMap, queue_size=1)
    get_start = rospy.Subscriber('initialpose', PoseWithCovarianceStamped, setStart, queue_size=1)
    get_goal = rospy.Subscriber('move_base_simple/goal', PoseStamped, setGoal, queue_size =1)

    #set up all of the publicaitons. (start, goal, expanded, frontier, path)
    pub_start = rospy.Publisher('/startpoint', GridCells)
    pub_goal = rospy.Publisher('/goalpoint', GridCells)
    pub_frontier = rospy.Publisher('/frontier',GridCells)
    pub_expanded = rospy.Publisher('/expanded', GridCells)
    pub_path = rospy.Publisher('/path', GridCells)
    pub_waypoints = rospy.Publisher('/waypoints', GridCells)


    # Use this command to make the program wait for some seconds
    rospy.sleep(rospy.Duration(1, 0))



    print "Starting pathfinder"

    #print out our debug map, startting by makeing a list of all of the wall locations
	#pubMap(pub_path, mapInfo, mapData)
    
    lastGoal = (-1,-1)
    lastStart = (-1,-1)

    newMap = obstacleExpansion(1, mapInfo, mapData,pub_waypoints)
    pubMap(pub_waypoints, mapInfo, newMap)

    		
    r = rospy.Rate(10)
    while not rospy.is_shutdown():
    	if (goal is not lastGoal) or (start is not lastStart):
    		lastStart = start
    		lastGoal = goal
    		paths = aStar(start, goal, mapInfo, mapData, pub_frontier, pub_expanded)
    		publishGridList(paths[0], mapInfo, pub_path)
    		#publishGridList(paths[1], mapInfo, pub_waypoints)

        r.sleep()

    print "exiting pathfinder"

    #go back up the stack and print all of the points where there is a 100 onto the map. See if the shape is rotated.
    #we will need to make all of those functions anyway.