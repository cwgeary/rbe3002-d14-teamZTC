#!/usr/bin/env python

#python imports
import rospy, tf, math

#Our imports
from lab5Helpers import *
from aStar import *
from movement import *

#ROS imports
from std_msgs.msg import Header
from nav_msgs.msg import OccupancyGrid #format for reading the map. Data is stored by row.
from nav_msgs.msg import GridCells #format for publishing to rviz display
from geometry_msgs.msg import PoseWithCovarianceStamped #format for reading the start and goal points
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from kobuki_msgs.msg import BumperEvent
from geometry_msgs.msg import Twist



#this is strictly path finding code. As this node will be used as such in later labs
#This node automaticly subcribes to the map topic, and  publishis a service called getPath. getPath takes a 
#starting point and a goal point and then publishes the frontier, and the optimal path.

def readMap(msg):
    global mapInfo #map information such as width and hight, and cell sizes.
    global mapData #the cells of the map, with 100 = impassable and 0 = empty, -1 = unexplored. 
    global pub_map
    global mapProcessed
    global odom_list
    global path


    #mapData = msg.data
    #mapInfo = msg.info

    (tran, rot) = odom_list.lookupTransform('map','base_footprint', rospy.Time(0))

    startPoint = Point()
    startPoint.x = tran[0]
    startPoint.y = tran[1]

    #print "resizing new map"
    resizedMap = mapResize(0.1, msg.info, msg.data)
    
    mapInfo = resizedMap.info
    mapData = resizedMap.data

    #print "expanding new map."
    mapData = obstacleExpansion(1, mapInfo, mapData)
    resizedMap.data = mapData


    print "publishing new map"
    pub_map.publish(resizedMap)

    #print "plan a new path."
    paths = aStar(globalToGrid(startPoint, mapInfo), goal, mapInfo, mapData, pub_frontier, pub_expanded)
    publishGridList(paths[0], mapInfo, pub_path)

    print "path in readmap"
    path = paths[0]

    mapProcessed = 1

    #if(len(mapData) != mapInfo.width * mapInfo.height):
    #    print "map size does not match data length."
    #    print len(mapData)

def setStart(msg):
    global start, pub_start, mapInfo, mapData

    #define a point object that will represent our start point on the grid
    point = msg.pose.pose.position #this needs to be adjuseted depending on how the gridcells object is using the point object.

    #set the starting point for the search to the gridpoint defined by the user mouse click.
    start = globalToGrid(point, mapInfo)

    #define a new gridcells object 
    gridCells = GridCells()
    
    #start construction our message
    gridCells.header = msg.header
    gridCells.cell_width = mapInfo.resolution
    gridCells.cell_height = mapInfo.resolution
    cells = [gridToGlobal(start, mapInfo)]
    gridCells.cells = cells

    #pub_start.publish(gridCells)
    #print "startpoint set at"
    #print start

def setGoal(msg):
    global goal, pub_goal, goalPoint
    point = msg.pose.position

    #set the goal point for the search to the gridpoint defined by the user mouse click.
    goal = globalToGrid(point, mapInfo)
    goalPoint = gridToGlobal(goal, mapInfo)

    #define a new gridcells object 
    gridCells = GridCells()
    
    #start construction our message
    gridCells.header = msg.header
    gridCells.cell_width = mapInfo.resolution
    gridCells.cell_height = mapInfo.resolution
    cells = [goalPoint]
    gridCells.cells = cells

    pub_goal.publish(gridCells)
    print "goal set at"
    print goal




# This is the program's main function
if __name__ == '__main__':
    # Change this node name to include your username
    rospy.init_node('Lab_4_node')

    global mapInfo, mapData, path
    global frontier, expanded, path, start, goal, startPoint, goalPoint
    global odom_list

    global pub_start, pub_goal, pub_frontier, pub_path, pub_expanded, pub_waypoints, pub_map, pub
    global mapProcessed

    mapProcessed = 0

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

    pub_map = rospy.Publisher('/newMap', OccupancyGrid)
    pub = rospy.Publisher('cmd_vel_mux/input/teleop', Twist)

    odom_list = tf.TransformListener()


    # Use this command to make the program wait for some seconds
    rospy.sleep(rospy.Duration(1, 0))


    print "Starting pathfinder"
    path = []
    lastGoal = (-1,-1)
    lastStart = (-2,-2)

    r = rospy.Rate(20)
    while not rospy.is_shutdown():

    	if len(path) > 0:
    		driveToPoint(gridToGlobal(path[-1], mapInfo),odom_list, pub)

        r.sleep()

    print "exiting pathfinder"
