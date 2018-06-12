#! /usr/bin/env python

# So, in the above Python script, we are generating a very simple logic:
# If there's no obstacle closer than 1 meter in front of the robot: move forward
# If there's an obstacle closer than 1 meter in front of the robot: turn left
# If there's an obstacle closer than 1 meter at the right side of the robot: turn left
# If there's an obstacle closer than 1 meter at the left side of the robot: turn right


import rospy
import math

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

def callback(msg):

	#If the distance to an obstacle in front of the robot is bigger than 1 meter or NaN, the robot will move forward	
	if msg.ranges[320] > 1 or math.isnan(msg.ranges[320]):
	    move.linear.x = 0.2
            move.angular.z = 0.0

	#If the distance to an obstacle in front of the robot is smaller than 1 meter, the robot will turn left
	if msg.ranges[320] < 1: 
	    move.linear.x = 0.0
	    move.angular.z = 0.2
        
	#If the distance to an obstacle at the left side of the robot is smaller than 0.3 meters, the robot will turn right

	if msg.ranges[639] < 0.3:
            move.linear.x = 0.0
	    move.angular.z = -0.2
        
	#If the distance to an obstacle at the right side of the robot is smaller than 0.3 meters, the robot will turn left
  	if msg.ranges[0] < 0.3:
	    move.linear.x = 0.0
	    move.angular.z = 0.2
      
  	pub.publish(move)

rospy.init_node("move_risc")
sub = rospy.Subscriber('/scan', LaserScan, callback)
pub = rospy.Publisher('cmd_vel_mux/input/teleop', Twist, queue_size = 1)
move = Twist()

rospy.spin()
