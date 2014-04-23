#!/usr/bin/env python

import rospy, tf
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from kobuki_msgs.msg import BumperEvent
import math
import time

#takes a global point and drives the motors until it reaches the point.
#build to be called in a loop, and to replace the driveToNextWaypoint function.
def driveToPoint(nextPoint, odom_list, pub):
    print "driving to point"

    twist = Twist()

    ap = 1.75
    tp = 1

    minAngle = 0.03
    maxTurnSpeed = .5
    minTurnSpeed = .025
    maxSpeed = .5
    minSpeed = .075

    #get the current pose of the robot from TF
    (tran, rot) = odom_list.lookupTransform('map','base_footprint', rospy.Time(0))
    euler = tf.transformations.euler_from_quaternion(rot)
    currentAngle = euler[2]

    print "current angle" + str(currentAngle)
    
    #find the angle of the target vector
    targetAngle = math.atan2(nextPoint.y - tran[1], nextPoint.x - tran[0])
    print "targetAngle" + str(targetAngle)

    deltaAngle = targetAngle - currentAngle

    #now calculate the angle between the current robot position and the vector from the current point to the target point
    turnTest = deltaAngle + 0.2
    
    print "deltaAngle" + str(deltaAngle)

    turnSpeed = ap*deltaAngle

    if angleGetDiff(turnTest) < angleGetDiff(currentAngle):
        turnSpeed = turnSpeed

    if abs(deltaAngle) < minAngle:
        turnSpeed = 0

    elif abs(turnSpeed) < minSpeed:
        if turnSpeed < 0:
            turnSpeed = -minTurnSpeed
        elif turnSpeed > 0:
            turnSpeed = minTurnSpeed

    elif turnSpeed > maxTurnSpeed:
        turnSpeed = maxTurnSpeed
    elif turnSpeed < -maxSpeed:
        turnSpeed = -maxTurnSpeed

    #now that we have a turning speed, find the linear distance
    distance = math.sqrt(math.pow(tran[0]-nextPoint.x,2) + math.pow(tran[1]-nextPoint.y,2))

    driveSpeed = distance * tp
    if driveSpeed > maxSpeed:
        driveSpeed = maxSpeed
    elif driveSpeed < minSpeed:
        driveSpeed = 0

    print "turn speed"
    print turnSpeed
    print "drive speed"
    print driveSpeed

    #now that we have our speeds sum them or trim them depending on what the current states are.
    if abs(deltaAngle) < minAngle:
        print "driving"
        twist.linear.x = driveSpeed; twist.linear.y = 0; twist.linear.z = 0
        twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0;
    else:
        print "turning"
        twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
        twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = turnSpeed;

    pub.publish(twist)


def angleGetDiff(deltaAngle):
    if deltaAngle > math.pi:
        deltaAngle = deltaAngle - 2*math.pi
    elif deltaAngle < -math.pi:
        deltaAngle = deltaAngle + 2*math.pi
    return deltaAngle