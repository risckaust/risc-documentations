#! /usr/bin/env python
# So, in the above Python script, we are generating a very simple logic:
# If there's no obstacle closer than 1 meter in front of the robot: move forward
# If there's an obstacle closer than 1 meter in front of the robot: turn left
# If there's an obstacle closer than 1 meter at the right side of the robot: turn left
# If there's an obstacle closer than 1 meter at the left side of the robot: turn right

import rospy
from sensor_msgs.msg import LaserScan 
from geometry_msgs.msg import Twist

def callback(msg): 
  print msg.ranges[360] #We print the distance to an obstacle in front of the robot

#If the distance to an obstacle in front of the robot is bigger than 1 meter, the robot will move forward
  if msg.ranges[360] > 1:
      move.linear.x = 0.1
      move.angular.z = 0.0

#If the distance to an obstacle in front of the robot is smaller than 1 meter, the robot will turn left
  if msg.ranges[360] < 1: 
      move.linear.x = 0.0
      move.angular.z = 0.2
        
#If the distance to an obstacle at the left side of the robot is smaller than 0.3 meters, the robot will turn right
  if msg.ranges[719] < 0.3:
      move.linear.x = 0.0
      move.angular.z = -0.2
        
#If the distance to an obstacle at the right side of the robot is smaller than 0.3 meters, the robot will turn left
  if msg.ranges[0] < 0.3:
      move.linear.x = 0.0
      move.angular.z = 0.2
      
  pub.publish(move)

rospy.init_node('sub_node')
sub = rospy.Subscriber('/kobuki/laser/scan', LaserScan, callback) #We subscribe to the laser's topic
pub = rospy.Publisher('/cmd_vel', Twist)
move = Twist()

rospy.spin()
