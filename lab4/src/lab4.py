#!/usr/bin/env python


import rospy, tf, math
from lab4Helpers import *
from aStar import *
#from movement import *
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

    print "resizing new map"
    resizedMap = mapResize(0.1, msg.info, msg.data)
    
    mapInfo = resizedMap.info
    mapData = resizedMap.data

    print "expanding new map."
    mapData = obstacleExpansion(2, mapInfo, mapData)
    resizedMap.data = mapData

    print "publishing new map"
    pub_map.publish(resizedMap)

    print "plan a new path."
    paths = aStar(start, goal, mapInfo, mapData, pub_frontier, pub_expanded)
    publishGridList(paths[0], mapInfo, pub_path)

    mapProcessed = 1

    if(len(mapData) != mapInfo.width * mapInfo.height):
        print "map size does not match data length."
        print len(mapData)

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

def readOdom(msg):
    global start, pub_start, mapInfo, mapData, pose, yaw
    pose = msg.pose.pose
    point = msg.pose.pose.position

    quaternion = (
        pose.orientation.x,
        pose.orientation.y,
        pose.orientation.z,
        pose.orientation.w)
    euler = tf.transformations.euler_from_quaternion(quaternion)
    yaw = euler[2]

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
    global goal, pub_goal
    point = msg.pose.position

    #set the goal point for the search to the gridpoint defined by the user mouse click.
    goal = globalToGrid(point, mapInfo)

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
    print "goal set at"
    print goal


def driveToNextWaypoint(path):
	global pose

	if len(path) == 0:
		return

	currentPoint = pose.position
	#get the global position of the next point in the path.
	nextPoint = gridToGlobal(path[0], mapInfo)

	#get the distance beween the current pos and the next waypoint
	distance = math.sqrt(math.pow(currentPoint.x-nextPoint.x,2) + math.pow(currentPoint.y-nextPoint.y,2))

	#angle between the start and end points in radians. Returns a value between pi and -pi
	pointAngle = math.atan2(currentPoint.x - nextPoint.x, currentPoint.y - nextPoint.y)

	#publish a twist message with the required inputs.
	rotate(pointAngle)
	driveStraight(0.2, distance)

def driveStraight(speed, distance):
    global pub
    global pose
    print 'driving straight'
    odist = 0

    r = rospy.Rate(50)

    startx = pose.position.x
    starty = pose.position.y
    while (pose.position.x-startx)**2 + (pose.position.y - starty)**2 < distance**2:
        twist = Twist()
        twist.linear.x = speed; twist.linear.y = 0; twist.linear.z = 0
        twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0;
        pub.publish(twist)
        r.sleep()
    
#Accepts an angle and makes lthe robot rotate that angle.
#the angle is passed in rad
def rotate(angle):
    print 'turning now...'
    global pub
    global yaw

    r = rospy.Rate(50)

    startAngle = yaw
    targetAngle = yaw + angle

    if targetAngle > math.pi:
        targetAngle = targetAngle - 2*math.pi
    elif targetAngle < -math.pi:
        targetAngle = targetAngle + 2*math.pi

    print targetAngle

    if(angle < 0):
        speed = -.5   
    else:
        speed = 0.5

    while abs(yaw - targetAngle) > 0.025:
        twist = Twist()
        twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
        twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = speed;
        pub.publish(twist)
        r.sleep()

# This is the program's main function
if __name__ == '__main__':
    # Change this node name to include your username
    rospy.init_node('Lab_4_node')

    global mapInfo, mapData
    global frontier, expanded, path, start, goal

    global pub_start, pub_goal, pub_frontier, pub_path, pub_expanded, pub_waypoints, pub_map, pub

    global mapProcessed
    mapProcessed = 0

    goal = (-1,-1)
    start = (-2,-2)

    #Set up the subscriptions to all of the nessaray data
    map_sum = rospy.Subscriber('map',OccupancyGrid, readMap, queue_size=1)
    get_start = rospy.Subscriber('initialpose', PoseWithCovarianceStamped, setStart, queue_size=1)
    get_goal = rospy.Subscriber('move_base_simple/goal', PoseStamped, setGoal, queue_size =1)
    get_pos = rospy.Subscriber('odom', Odometry, readOdom)

    #set up all of the publicaitons. (start, goal, expanded, frontier, path)
    pub_start = rospy.Publisher('/startpoint', GridCells)
    pub_goal = rospy.Publisher('/goalpoint', GridCells)
    pub_frontier = rospy.Publisher('/frontier',GridCells)
    pub_expanded = rospy.Publisher('/expanded', GridCells)
    pub_path = rospy.Publisher('/path', GridCells)
    pub_waypoints = rospy.Publisher('/waypoints', GridCells)

    pub_map = rospy.Publisher('/newMap', OccupancyGrid)
    pub = rospy.Publisher('cmd_vel_mux/input/teleop', Twist)


    # Use this command to make the program wait for some seconds
    rospy.sleep(rospy.Duration(5, 0))



    print "Starting pathfinder"

    
    lastGoal = (-1,-1)
    lastStart = (-2,-2)

    r = rospy.Rate(10)
    while not rospy.is_shutdown():
    	if ((goal is not lastGoal) or (start is not lastStart)) and (mapProcessed == 1):
    		lastStart = start
    		lastGoal = goal
    		paths = aStar(start, goal, mapInfo, mapData, pub_frontier, pub_expanded)
    		publishGridList(paths[0], mapInfo, pub_path)
    		publishGridList(paths[1], mapInfo, pub_waypoints)

    		driveToNextWaypoint(paths[0])

        r.sleep()

    print "exiting pathfinder"

    #go back up the stack and print all of the points where there is a 100 onto the map. See if the shape is rotated.
    #we will need to make all of those functions anyway.

