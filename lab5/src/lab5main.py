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
def aStar_client(goal):
    rospy.wait_for_service('aStar')
    try:
        aStar = rospy.ServiceProxy('aStar', AStar)
        resp = aStar(goal.x, goal.y)

        waypoints = []
        p = Point()

        for n in resp.xList
            p.x = resp.xList[n]
            p.y = resp.yList[n]
            waypoints[n] = p

        return waypoints
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

def getGoal(msg):
    global goal
    goal = msg
# This is the program's main function
if __name__ == '__main__':
    rospy.init_node('Lab_5_node')
    global goal
    goal = (-1, -1)

    #set up all of the publicaitons. (start, goal, expanded, frontier, path)
    goal_receive = rospy.Subscriber('waypoint',Point, getGoal, queue_size=1)
    pub = rospy.Publisher('cmd_vel_mux/input/teleop', Twist)

    # Use this command to make the program wait for some seconds
    rospy.sleep(rospy.Duration(1, 0))

    r = rospy.Rate(20)
    while not rospy.is_shutdown():
        waypoints = aStar_client(goal)
    	if len(path) > 0:
    		driveToPoint(waypoints[1])

        r.sleep()

    print "exiting pathfinder"
