#! /usr/bin/env python
##@package final_assignment
#This node is in charge of monitoring the state of the robot, and activate the corresping services and behaviour.
#
#As already said this node activates the corresponding services via the function change_state(state),
#and for the first two behaviours it sets the new goal position with the function new_goal().

import rospy
import time

from geometry_msgs.msg import Point
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from tf import transformations

from std_srvs.srv import *
from geometry_msgs.msg import Twist
from move_base_msgs.msg import MoveBaseActionGoal
from actionlib_msgs.msg import GoalID
import math

##Publisher that publishes on /cmd_vel to stop the robot when entering in state of 'stop'.
twist_pub= None
##Publisher that publish a MoveBaseActionGoal msg on /move_base/goal to make the robot reach the goal.
mvgoal_pub=None
##Publisher that publish a GoalID msg on /move_base/cancel to cancel a previous goal.
mvcancel_pub=None

##Client for activating the service for wall following behaviour.
srv_client_wall_follower_ = None
##Client for activating the service calling the user main user interface.
srv_client_user_interface_assignment_=None
##CLient for activating the bug navigation behaviour.
srv_client_bug_algorithm_ = None

##Variable containing position of the robot.
position_ = Point()
##Variables for controlling laser data
regions_ = None
##Variable for checking if a new state has been issued
change_=0;
##States of the robot definiton
state_desc_ = ['random target', 'get target', 'wall fallowing','stop','bug']
##Variable cointaining the actual state of the robot
state_ = 0
##Variable containing the new state of the robot
newstate_= 0
##Variable to control the different user interface when the behaviour is switched to bug.
user_=0

##Callback for keeping track of robot pose.
#
#Callback for subscriber of /odom for checking robot pose.
#@param msg geometry_msgs/Odometry
def clbk_odom(msg):
    global position_

    position_ = msg.pose.pose.position

##Function for computing the distance from the robot actual position to the desired position
def distance():
    d=math.sqrt(pow(rospy.get_param('des_pos_y') - position_.y, 2) +
                        pow(rospy.get_param('des_pos_x')- position_.x, 2))
    return d

##Function for setting a new target with a MoveBaseActionGoal msg on topic /movebase/goal
def new_goal():
    msg_goal=MoveBaseActionGoal()
    msg_goal.goal.target_pose.header.frame_id="map"
    msg_goal.goal.target_pose.pose.orientation.w=1;
    msg_goal.goal.target_pose.pose.position.x=rospy.get_param('des_pos_x');
    msg_goal.goal.target_pose.pose.position.y=rospy.get_param('des_pos_y');
    mvgoal_pub.publish(msg_goal)
    print("goal: x=" + str(rospy.get_param('des_pos_x')) + " y=" + str(rospy.get_param('des_pos_y')))

##Function for changing the state of the robot
#
#This function changes the state of the robot activating the various services and the respectfull operations
#set only the request for service for first behaviour to True 
#Set wall follow and bug algorithm behaviour to false
#call new goal for setting the new target
def change_state(state):
    global state_, state_desc_, twist_pub
    global srv_client_wall_follower_,srv_client_user_interface_assignment_,srv_client_bug_algorithm_
    
    rospy.set_param("change",0)
    log = "state changed in: %s" % state_desc_[state]
    rospy.loginfo(log)
    if state == 0:
        
        resp = srv_client_wall_follower_(False)
        resp=srv_client_bug_algorithm_(False)

	new_goal()

    if state == 1:
	resp = srv_client_wall_follower_(False)
        resp=srv_client_bug_algorithm_(False)

	new_goal()
        
    if state == 2:
        resp = srv_client_wall_follower_(True)
        resp=srv_client_bug_algorithm_(False)

        resp = srv_client_user_interface_assignment_()

    if state == 3:
	resp = srv_client_wall_follower_(False)
        resp = srv_client_bug_algorithm_(False)
	twist_msg = Twist()
        twist_msg.linear.x = 0
        twist_msg.linear.y = 0
        twist_msg.angular.z = 0
        twist_pub.publish(twist_msg)
        print("robot stopped at position: x= "+ str(position_.x) +" y=" + str(position_.y))
       
        resp = srv_client_user_interface_assignment_()

    if state == 4:
	resp = srv_client_wall_follower_(False)
        resp = srv_client_bug_algorithm_(True)

    rospy.set_param("state",state)

##Main function
#
#initializes necessary variables, works as a state machine
def main():
    global position_, state_,newstate_,change_,counter
    global srv_client_wall_follower_, srv_client_user_interface_assignment_,srv_client_bug_algorithm_ 
    global twist_pub,mvgoal_pub,mvcancel_pub

    rospy.init_node('big_brain')

    sub_odom = rospy.Subscriber('/odom', Odometry, clbk_odom)
    twist_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    mvgoal_pub = rospy.Publisher('/move_base/goal', MoveBaseActionGoal, queue_size=1)
    mvcancel_pub = rospy.Publisher('/move_base/cancel', GoalID, queue_size=1)

    #service client for user interface
    srv_client_user_interface_assignment_ = rospy.ServiceProxy('/user_interface_assignment', Empty)
    #service client for wall follower behaviour
    srv_client_wall_follower_ = rospy.ServiceProxy('/wall_follower_switch', SetBool)
    #service client for bug algorithm
    srv_client_bug_algorithm_ = rospy.ServiceProxy('/bug_algorithm_srv', SetBool)

    # initialize as stopped
    change_state(3)


    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
	state_=rospy.get_param("state")
    	change_=rospy.get_param("change")
	newstate_=rospy.get_param("newstate")

        if state_ == 0 or state_== 1:
            dis=distance()
            print("distance from goal:",dis)   
            if dis<0.3:
                 msg_cancel=GoalID()
                 mvcancel_pub.publish(msg_cancel)
		 print("goal reached")    
                 change_state(3)

        elif state_ == 2 or state_ == 3 or state_ == 4:
            if change_ == 1:
	         msg_cancel=GoalID()
                 mvcancel_pub.publish(msg_cancel)    
                 change_state(newstate_)
	    else:
		 continue	 

        rate.sleep()


if __name__ == "__main__":
    main()

