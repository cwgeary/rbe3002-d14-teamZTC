#!/usr/bin/env python

import rospy, tf
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from kobuki_msgs.msg import BumperEvent
import math
import time
from lab5Helpers import *

#takes a list of global points and drives the motors until it reaches the closest point that is the 
#furtherst along the path. built to be called in a loop, and to replace the driveToNextWaypoint function.
def driveToPoint(speed, path, odom_list, pub, debugPub, mapInfo, mapData):
    #print "driving to point"

    twist = Twist()

    ap = 5
    tp = 5

    minAngle = 0.03
    minDistance = 0.75

    maxTurnSpeed = 1
    minTurnSpeed = .5
    maxSpeed = .35
    minSpeed = .2

    #get the current pose of the robot from TF
    (tran, rot) = odom_list.lookupTransform('map','base_footprint', rospy.Time(0))
    euler = tf.transformations.euler_from_quaternion(rot)
    currentAngle = euler[2]
    currentPoint = Point()
    currentPoint.x = tran[0]
    currentPoint.y = tran[1]

    #get the next point in the sequence
    #nextPoint = getNextPoint(path, currentPoint, mapInfo, mapData, debugPub)
    nextPoint = path[-1]


    #now that we have a next point, find the linear distance
    distance = math.sqrt(math.pow(tran[0]-nextPoint.x,2) + math.pow(tran[1]-nextPoint.y,2))
    distanceTwo = 0
    if len(path) > 1:
        distanceTwo = math.sqrt(math.pow(tran[0]-nextPoint.x,2) + math.pow(tran[1]-nextPoint.y,2))

        if distance > distanceTwo:
            nextPoint = path[-2]
            print "skipping"
            distance = distanceTwo

    #print "current angle" + str(currentAngle)
    
    #find the angle of the target vector
    targetAngle = math.atan2(nextPoint.y - tran[1], nextPoint.x - tran[0])
    #print "targetAngle" + str(targetAngle)

    deltaAngle = targetAngle - currentAngle

    #now calculate the angle between the current robot position and the vector from the current point to the target point
    turnTest = deltaAngle + 0.2
    
    #print "deltaAngle" + str(deltaAngle)

    turnSpeed = ap*deltaAngle

    if angleGetDiff(turnTest) < angleGetDiff(currentAngle):
        turnSpeed = turnSpeed

    if abs(deltaAngle) < minAngle:
        turnSpeed = 0

    elif abs(turnSpeed) < minSpeed:
        print "speed is below minimum"
        if turnSpeed < 0:
            turnSpeed = -minTurnSpeed
        elif turnSpeed > 0:
            turnSpeed = minTurnSpeed

    elif turnSpeed > maxTurnSpeed:
        turnSpeed = maxTurnSpeed
    elif turnSpeed < -maxSpeed:
        turnSpeed = -maxTurnSpeed

    driveSpeed = distance * tp
    if driveSpeed > maxSpeed:
        driveSpeed = maxSpeed
    elif driveSpeed < minSpeed:
        driveSpeed = minSpeed

    print "target turn speed: " + str(turnSpeed)
    print "target drive speed: " + str(driveSpeed)

    if abs(driveSpeed - speed[0]) >= 0.05:
        if driveSpeed > speed[0]:
            driveSpeed = speed[0] + 0.05
        elif driveSpeed < speed[0]:
            driveSpeed = speed[0] - 0.05

    if abs(turnSpeed - speed[1]) >= 0.05:
        if turnSpeed > speed[1]:
            turnSpeed = speed[1] + 0.05
        elif turnSpeed < speed[1]:
            turnSpeed = speed[1] - 0.05

    #now that we have our speeds sum them or trim them depending on what the current states are.
    if abs(deltaAngle) < minAngle:
        #print "driving"
        twist.linear.x = driveSpeed; twist.linear.y = 0; twist.linear.z = 0
        twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0;
    else:
        #print "turning"
        twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
        twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = turnSpeed;

    print "turn speed: " + str(turnSpeed)
    print "drive speed: " + str(driveSpeed)

    pub.publish(twist)
    return (driveSpeed, turnSpeed)


def angleGetDiff(deltaAngle):
    if deltaAngle > math.pi:
        deltaAngle = deltaAngle - 2*math.pi
    elif deltaAngle < -math.pi:
        deltaAngle = deltaAngle + 2*math.pi
    return deltaAngle

def getNextPoint(waypointList, current, mapInfo, mapData, pub):
    nextPoint = Point()
    sortedPoints = []

    for p in waypointList:
        distance = ecludianPointDist(p, current)

        sortedPoints.append((distance*checkVis(current, p, mapInfo, mapData, pub), p))

    return max(sortedPoints,key=lambda item:item[0])[1]

def robotSpin(pub):
    currentTime = rospy.get_time()
    a = 0
    if int(currentTime) % 5 == 0:
        a = 0
    else:
        a = 1

    #print "a: " + str(a) 

    twist = Twist()
    twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
    twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = a;
    pub.publish(twist)