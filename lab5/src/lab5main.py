#!/usr/bin/env python

#python imports
import rospy, tf, math

#Our imports
from lab5Helpers import *
from aStar import *
from movement import *
from lab5.srv import *

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
        
        for n in range(len(resp.pathX)):
            p = Point()
            p.x = resp.pathX[n]
            p.y = resp.pathY[n]
            waypoints.append(p)
        #print "we got da points! " + str(waypoints)
        return waypoints
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

def getGoal(msg):
    global goal
    goal = msg


def timerCallback(event):
    global waypoints, goal
    waypoints = aStar_client(goal)


# This is the program's main function
if __name__ == '__main__':
    rospy.init_node('Lab_5_node')
    
    global goal, waypoints
    goal = Point()
    waypoints = []
    odom_list = tf.TransformListener()

    #set up all of the publicaitons. (start, goal, expanded, frontier, path)
    goal_receive = rospy.Subscriber('waypoint', Point, getGoal, queue_size=1)
    pub = rospy.Publisher('cmd_vel_mux/input/teleop', Twist)
    pub_drivedebug = rospy.Publisher('/ddebug', GridCells)


    rospy.Timer(rospy.Duration(1), timerCallback)

    # Use this command to make the program wait for some seconds
    rospy.sleep(rospy.Duration(1, 0))

    r = rospy.Rate(20)
    while not rospy.is_shutdown():
    	if len(waypoints) > 0:
            #print "drive waypoints: " + str(waypoints)
            driveToPoint(waypoints, odom_list, pub)

        r.sleep()

    print "exiting pathfinder"

