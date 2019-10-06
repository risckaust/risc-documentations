#!/usr/bin/env python

# ROS python API
import rospy
# 3D point & Stamped Pose msgs
from geometry_msgs.msg import Point, PoseStamped,  Quaternion
# import all mavros messages and services
from mavros_msgs.msg import *
from mavros_msgs.srv import *
import math
from tf.transformations import quaternion_from_euler
from time import sleep

# Modes are activated using ROS services

def setArm():
    rospy.wait_for_service('mavros/cmd/arming')
    try:
        armService = rospy.ServiceProxy('mavros/cmd/arming', mavros_msgs.srv.CommandBool)
        armService(True)
    except rospy.ServiceException, e:
        print "Service arming call failed: %s"%e

def setDisarm():
    rospy.wait_for_service('mavros/cmd/arming')
    try:
        armService = rospy.ServiceProxy('mavros/cmd/arming', mavros_msgs.srv.CommandBool)
        armService(False)
    except rospy.ServiceException, e:
        print "Service disarming call failed: %s"%e

def setOffboardMode():
    rospy.wait_for_service('mavros/set_mode')
    try:
        flightModeService = rospy.ServiceProxy('mavros/set_mode', mavros_msgs.srv.SetMode)
        flightModeService(custom_mode='OFFBOARD')
    except rospy.ServiceException, e:
        print "service set_mode call failed: %s. Offboard Mode could not be set."%e


class Controller:
    # Initialization method
    def __init__(self):
        self.state               = State() # Rover state
        self.sp                  = PoseStamped() # Instantiate a setpoint message
        yaw_degrees              = 0  # North
        yaw                      = math.radians(yaw_degrees)
        quaternion               = quaternion_from_euler(0, 0, yaw)
        self.sp.pose.orientation = Quaternion(*quaternion)       
        self.local_pos           = Point(0.0, 0.0, 0.0) # Local position of the rover
        self.counter             = 0 # Counter to keep the number of waypoints reached
        self.waypointsx          = [50,50,0,0] # Waypoints
        self.waypointsy          = [0,50,50,0] # Waypoints
    
    ##### Callbacks #####

    ## Local position callback
    def posCb(self, msg):
        self.local_pos.x = msg.pose.position.x
        self.local_pos.y = msg.pose.position.y
        self.local_pos.z = msg.pose.position.z


    ## Rover State callback
    def stateCb(self, msg):
        self.state = msg

    ## Update setpoint message
    def updateSp(self):

        self.sp.pose.position.x = self.waypointsx[self.counter]
        self.sp.pose.position.y = self.waypointsy[self.counter]
        if (abs(self.local_pos.x-self.sp.pose.position.x)<1.5 and abs(self.local_pos.y-self.sp.pose.position.y)<1.5):
        # Once waypoint reached within 1.5m, counter is increased and we change to next waypoint
            self.counter=self.counter+1
            print self.counter

# Main function
def main():

    # Initiate node
    rospy.init_node('node', anonymous=True)

    # Rover object
    rover = Controller()

    # ROS loop rate, [Hz]
    rate = rospy.Rate(20.0)

    # Subscribe to state
    rospy.Subscriber('mavros/state', State, rover.stateCb)

    # Subscribe to local position
    rospy.Subscriber('mavros/local_position/pose', PoseStamped, rover.posCb)

    # Setpoint publisher
    sp_pub = rospy.Publisher('mavros/setpoint_position/local', PoseStamped , queue_size=1)


    # Make sure the rover is armed
    while not rover.state.armed:
        setArm()
        rate.sleep()

    # We need to send few setpoint messages, then activate OFFBOARD mode, to take effect
    k=0
    while k<10:
        sp_pub.publish(rover.sp)
        rate.sleep()
        k = k+1

    # Activate OFFBOARD mode
    setOffboardMode()

    # ROS main loop
    while not rospy.is_shutdown():
        if (rover.counter < 4):
            rover.updateSp()
        else:
            setDisarm()
        sp_pub.publish(rover.sp)

        rate.sleep()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass