#! /usr/bin/env python

##@package final_assignment
#This node implements the bug algoritm for navigation
#
#It's used to manage the output on shell 
# when the bug algoritm is activated

import rospy
import time
# import ros message
from geometry_msgs.msg import Point
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from tf import transformations
# import ros service
from std_srvs.srv import *
from geometry_msgs.msg import Twist

import math
#initialize needed global variables
active_ = False
start_time=0
elapsed_time=0
actual_time=0
#global publisher
pub = None
#global service client
srv_client_go_to_point_ = None
srv_client_wall_follower_ = None
srv_client_user_interface_ = None
srv_client_user_interface_assignment_ = None
#global variables
yaw_ = 0
yaw_error_allowed_ = 5 * (math.pi / 180)  # 5 degrees
position_ = Point()
desired_position_ = Point()
desired_position_.x = rospy.get_param('des_pos_x')
desired_position_.y = rospy.get_param('des_pos_y')
desired_position_.z = 0
regions_ = None
#possible state
state_desc_ = ['Go to point', 'wall following', 'stopped']
state_ = 0
# 0 - go to point
# 1 - wall following
# 2 - stopped

##Function to retrieve robot pose
#
#it's the callback for the subscriber on topic /odom
def clbk_odom(msg):
    global position_, yaw_

    # position
    position_ = msg.pose.pose.position

    # yaw
    quaternion = (
        msg.pose.pose.orientation.x,
        msg.pose.pose.orientation.y,
        msg.pose.pose.orientation.z,
        msg.pose.pose.orientation.w)
    euler = transformations.euler_from_quaternion(quaternion)
    yaw_ = euler[2]

##Laser data callback
#
#output are localized in the robot in 5 regions organized as at the bottom
#variable regions_ separes the output of lasers
def clbk_laser(msg):
    global regions_
    regions_ = {
        'right':  min(min(msg.ranges[0:143]), 10),
        'fright': min(min(msg.ranges[144:287]), 10),
        'front':  min(min(msg.ranges[288:431]), 10),
        'fleft':  min(min(msg.ranges[432:575]), 10),
        'left':   min(min(msg.ranges[576:719]), 10),
    }

##Function activated on service request
#
#@param req is the request of the service
def bug_algorithm_srv(req):
    #behaviour of bug algorithm is activated according to the request of the service
    global active_
    active_ = req.data
    res = SetBoolResponse()
    res.success = True
    res.message = 'Done!'
    start_time=time.clock()
    rospy.set_param("start_time", start_time)
    rospy.set_param("user", 0)
   
    #each time the service is requested (active_ true or false) the robot is stopped
    change_state(2)
    return res

##Change state function
#
#manage the change of status of the robot,
#activating the corresponding services
#@param state is the desired state
def change_state(state):
    global state_, state_desc_
    global srv_client_wall_follower_, srv_client_go_to_point_,pub
    state_ = state

    if state_ == 0:
        #go to point behaviour, related service is set to true the other is set to false
        log = "state changed: %s" % state_desc_[state]
        rospy.loginfo(log)
        resp = srv_client_go_to_point_(True)
        resp = srv_client_wall_follower_(False)

    if state_ == 1:
        #wall follow behaviour, related service is set to true the other is set to false
        log = "state changed: %s" % state_desc_[state]
        rospy.loginfo(log)
        resp = srv_client_go_to_point_(False)
        resp = srv_client_wall_follower_(True)

    if state_ == 2:
        resp = srv_client_go_to_point_(False)
        resp = srv_client_wall_follower_(False)
        #a message Twist() is published on topic /cmd_vel for stop the robot
        twist_msg = Twist()
        twist_msg.linear.x = 0
        twist_msg.angular.z = 0
        pub.publish(twist_msg)
	print("robot stopped at position: x= "+ str(position_.x) +" y=" + str(position_.y))
        start_time=time.clock()
        rospy.set_param("start_time", start_time)

##Normalize angle function
#
#normalizes an angle
def normalize_angle(angle):
    if(math.fabs(angle) > math.pi):
        angle = angle - (2 * math.pi * angle) / (math.fabs(angle))
    return angle

##Main function
#
#Initalizes all the necessary pubs, subs ,clients and server
#It basically works as state machine with in addition some controls
#about the elapsed time that might result in aborting the reaching
def main():
    global active_,start_time,elapsed_time,actual_time,user
    global regions_, position_, desired_position_, state_, yaw_, yaw_error_allowed_
    global srv_client_go_to_point_, srv_client_wall_follower_, srv_client_user_interface_, pub, srv_client_user_interface_assignment_

    #init node
    rospy.init_node('bug0')
    srv = rospy.Service('bug_algorithm_srv', SetBool, bug_algorithm_srv)
    sub_laser = rospy.Subscriber('/scan', LaserScan, clbk_laser)
    sub_odom = rospy.Subscriber('/odom', Odometry, clbk_odom)
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    
    #service clients for: go to point behaviour, wall follow behaviour, user interface to set the position, user interface to change target
    srv_client_go_to_point_ = rospy.ServiceProxy('/go_to_point_switch', SetBool)
    srv_client_wall_follower_ = rospy.ServiceProxy('/wall_follower_switch_bug', SetBool)
    srv_client_user_interface_ = rospy.ServiceProxy('/user_interface', Empty)
    srv_client_user_interface_assignment_ = rospy.ServiceProxy('/user_interface_assignment', Empty)
    # initialize stopped
    change_state(2)
    count=0
    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
    	if not active_:
		continue
	else:    
        	#get the actual time and compare with the start time of reaching the point
                #compute the elapsed time
		count=count + 1
		actual_time=time.clock()
		elapsed_time=(actual_time*10)-(start_time*10)
		print("elapsed time :", elapsed_time)
		print("count :", count)
		if elapsed_time>100 and count>1:
                	#if the elapsed time is greater than 100 the goal is not reacheable 
                        #service is disabled
                        #robot is stopped
                        #interface is called
                        rospy.set_param("user", 0)
                        elapsed_time=0
			print("not reacheable goal")
			active_= False
			change_state(2)
			print("please insert a new target, insert again 5 if you want to return tu bug algorithm")
			resp=srv_client_user_interface_assignment_()
                else:
                                                        
                	#if no obstacles continue
			if regions_ == None:
				continue
                        #go to point behaviour
			if state_ == 0:
                        	#check the distance from the goal
				err_pos = math.sqrt(pow(desired_position_.y - position_.y,2) + pow(desired_position_.x - position_.x, 2))
                                #if distance smaller than 0.3 goal is reached and robot is stopped
				if(err_pos < 0.3):
                        		rospy.set_param("user", 1)
					change_state(2)
                                #if in front of robot there is a wall robot start wall follow
				elif regions_['front'] < 0.5:
					change_state(1)
                        #wall follower behaviour
                        elif state_ == 1:
                        	#compute desired orientation of the robot
				desired_yaw = math.atan2(desired_position_.y - position_.y, desired_position_.x - position_.x)
                                #compute the error with respect to the orientation and to the distance
				err_yaw = normalize_angle(desired_yaw - yaw_)
				err_pos = math.sqrt(pow(desired_position_.y - position_.y,2) + pow(desired_position_.x - position_.x, 2))
                                #if distance smaller than 0.3 goal is reached and robot is stopped
				if(err_pos < 0.3):
					rospy.set_param("user", 1)
					change_state(2)
                                #if there are no obstacles in front of the robot and orientation of robot is correct pass to go to point
				if regions_['front'] > 1 and math.fabs(err_yaw) < 0.05:
								change_state(0)
                        #robot stopped
			elif state_ == 2:
                                user=rospy.get_param("user")
                                if user==1:
                                	#if user is 1 means that the desired position has been reched
                                        #call the general user interface user can chose again to state in bug
                                        # algorithm or change behaviour
                                        rospy.set_param("user",0) 
                                        print("target reached")
                                        print("if you want to return again to bug algorithm chose 5 as target")
                                        resp=srv_client_user_interface_assignment_()
                        	#if the service is active and the robot is stopped a new goal position is asked to robot
				else:
					resp=srv_client_user_interface_()
					time.sleep(1)
                                        #take the goal position
					desired_position_.x = rospy.get_param('des_pos_x')
					desired_position_.y = rospy.get_param('des_pos_y')
					err_pos = math.sqrt(pow(desired_position_.y-position_.y,2)+pow(desired_position_.x-position_.x, 2))
                                        #if distance from desired position is grater to 0.35 it pass togo to point
                                        #otherwise it means that the robot is already in target position so asked for a new goal
					if(err_pos > 0.35):
						#
						start_time=time.clock()
						change_state(0)

	rate.sleep()



if __name__ == "__main__":
    main()
