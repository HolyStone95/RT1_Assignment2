#! /usr/bin/env python

##@package final_assignment
#This node implements the wall follower algoritm for navigation.
#
#It works specifically when the bug algoritm is NOT active

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf import transformations
from std_srvs.srv import *

import math

active_ = False
#publisher
pub_ = None
#laser output initialization
regions_ = {
    'right': 0,
    'fright': 0,
    'front': 0,
    'fleft': 0,
    'left': 0,
}
#possible states
state_ = 0
state_dict_ = {
    0: 'find the wall',
    1: 'turn left',
    2: 'follow the wall',
}

##Function activated on service request
#
#@param req is the request of the service
def wall_follower_switch(req):
    global active_
    active_ = req.data
    res = SetBoolResponse()
    res.success = True
    res.message = 'Done!'
    return res

##Laser data callback
#output are localized in the robot in 5 regions organized as at the bottom
#
#variable regions_ separes the output of lasers
def clbk_laser(msg):
    global regions_
    regions_ = {
        'right':  min(min(msg.ranges[0:143]), 10),
        'fright': min(min(msg.ranges[144:287]), 10),
        'front':  min(min(msg.ranges[288:431]), 10),
        'fleft':  min(min(msg.ranges[432:575]), 10),
        'left':   min(min(msg.ranges[576:713]), 10),
    }

    take_action()

##Change state function
#
#manage the change of status of the robot
def change_state(state):
    global state_, state_dict_
    if state is not state_:
        print ('Wall follower - [%s] - %s' % (state, state_dict_[state]))
        state_ = state

##Take action function
#
#checks the output of laser for knowing where are the position of wall 
#with respect to oriantation and position of robot
#the robot change state according where wall are
def take_action():
    global regions_
    regions = regions_
    msg = Twist()
    linear_x = 0
    angular_z = 0
    state_description = ''

    d0 = 1
    d = 1.5

    if regions['front'] > d0 and regions['fleft'] > d and regions['fright'] > d:
        state_description = 'case 1 - nothing'
        change_state(0)
    elif regions['front'] < d0 and regions['fleft'] > d and regions['fright'] > d:
        state_description = 'case 2 - front'
        change_state(1)
    elif regions['front'] > d0 and regions['fleft'] > d and regions['fright'] < d:
        state_description = 'case 3 - fright'
        change_state(2)
    elif regions['front'] > d0 and regions['fleft'] < d and regions['fright'] > d:
        state_description = 'case 4 - fleft'
        change_state(0)
    elif regions['front'] < d0 and regions['fleft'] > d and regions['fright'] < d:
        state_description = 'case 5 - front and fright'
        change_state(1)
    elif regions['front'] < d0 and regions['fleft'] < d and regions['fright'] > d:
        state_description = 'case 6 - front and fleft'
        change_state(1)
    elif regions['front'] < d0 and regions['fleft'] < d and regions['fright'] < d:
        state_description = 'case 7 - front and fleft and fright'
        change_state(1)
    elif regions['front'] > d0 and regions['fleft'] < d and regions['fright'] < d:
        state_description = 'case 8 - fleft and fright'
        change_state(1)
    else:
        state_description = 'unknown case'
        rospy.loginfo(regions)

##Function to find a wall
#
#randomly searches for walls
def find_wall():
    msg = Twist()
    msg.linear.x = 0.3
    msg.angular.z = -0.6
    return msg

##Turn left function
def turn_left():
    msg = Twist()
    msg.angular.z = 0.8
    return msg

##Follow the wall function
def follow_the_wall():
    global regions_

    msg = Twist()
    msg.linear.x = 0.5
    return msg

##Main function
#
#initializes necessary variables, works as a state machine 
def main():
    global pub_, active_
    #node
    rospy.init_node('wall_follower_main')
    #publisher of topic /cmd_vel for setting velocity
    pub_ = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    #subscriber of topic /scan for getting the laser scan
    sub = rospy.Subscriber('/scan', LaserScan, clbk_laser)
    #service for wall follow
    srv = rospy.Service('wall_follower_switch', SetBool, wall_follower_switch)

    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        if not active_:
            rate.sleep()
            continue
        else:
            #check the status
            #call the needed function to simulate a ceratain behaviour
            #function return the message that must be published on /cmd_vel
            msg = Twist()
            if state_ == 0:
                msg = find_wall()
            elif state_ == 1:
                msg = turn_left()
            elif state_ == 2:
                msg = follow_the_wall()
            else:
                rospy.logerr('Unknown state!')

            pub_.publish(msg)

        rate.sleep()


if __name__ == '__main__':
    main()
