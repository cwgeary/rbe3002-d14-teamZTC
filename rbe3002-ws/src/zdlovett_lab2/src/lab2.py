#!/usr/bin/env python

import rospy, tf
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from kobuki_msgs.msg import BumperEvent
import math
import time

#This function sequentially calls methods to perform a trajectory.
def executeTrajectory():
    while not bumperWasPressed:
        pass
    driveStraight(.5, .6)
    rotate(-1.57)
    driveArc(.15, .1, -3.13)
    rotate(2.35)
    driveStraight(.5, .42)



#This function accepts two wheel velocities and a time interval.
#wheel velocites are angular
def spinWheels(u1, u2, timeToRun):
    global pub
    print 'sending command spinWheels'
    t = 0
    r = rospy.Rate(10) #run at 10 Hz
    while t < timeToRun:
        t = t + 1
        r.sleep()
        twist = Twist()
        twist.linear.x = 0.5*wheelRadius*(u1+u2); twist.linear.y = 0; twist.linear.z = 0
        twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = (wheelRadius/wheelBase)(u1-u2)
        pub.publish(twist)


#This function accepts a speed and a distance for the robot to move in a straight line
#speed is given in m/s and distance is given in meters
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
        
#This function works the same as rotate how ever it does not publish linear velocities.
def driveArc(radius, speed, angle):
    print 'starting arc'
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

    while abs(yaw - targetAngle) > 0.025:
        twist = Twist()
        twist.linear.x = speed; twist.linear.y = 0; twist.linear.z = 0
        twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = speed/radius;
        pub.publish(twist)
        r.sleep()



#Odometry Callback function.
def read_odometry(msg):
    global pose
    global deltaTwist
    global yaw
    pose = msg.pose.pose
    
    #code found at http://answers.ros.org/question/69754/quaternion-transformations-in-python/    
    #type(pose) = geometry_msgs.msg.Pose
    quaternion = (
        pose.orientation.x,
        pose.orientation.y,
        pose.orientation.z,
        pose.orientation.w)
    euler = tf.transformations.euler_from_quaternion(quaternion)
    yaw = euler[2]

#Bumper Event Callback function
def readBumper(msg):
    global bumperWasPressed
    if (msg.state == 1):
        print 'I think we hit something...'
        bumperWasPressed = 1
        #use this to start the trajectory exicution later



# (Optional) If you need something to happen repeatedly at a fixed interval, write the code here.
# Start the timer with the following line of code: 
#   rospy.Timer(rospy.Duration(.01), timerCallback)
def timerCallback(event):
    pass # Delete this 'pass' once implemented


# This is the program's main function
if __name__ == '__main__':
    # Change this node name to include your username
    rospy.init_node('zdlovett_Lab_2_node')


    # These are global variables. Write "global <variable_name>" in any other function
    #  to gain access to these global variables
    global odomTwist
    global pub
    global pose
    global odom_tf
    global odom_list
    global wheelRadius
    global wheelBase
    global bumperWasPressed

    bumperWasPressed = 0;

    wheelBase = 250 / 1000
    wheelRadius = 38.1 / 1000
    
    # Replace the elipses '...' in the following lines to set up the publishers and subscribers the lab requires
    pub = rospy.Publisher('cmd_vel_mux/input/teleop', Twist) # Publisher for commanding robot motion
    sub = rospy.Subscriber('odom', Odometry, read_odometry, queue_size=1) # Callback function to read in robot Odometry messages

    bumper_sub = rospy.Subscriber('mobile_base/events/bumper', BumperEvent, readBumper, queue_size=1) # Callback function to handle bumper events

    # Use this object to get the robot's Odometry 
    odom_list = tf.TransformListener()
    
    # Use this command to make the program wait for some seconds
    rospy.sleep(rospy.Duration(1, 0))


    print "Starting Lab 2"

    # Make the robot do stuff..
    executeTrajectory()
    


    print "Lab 2 complete!"
