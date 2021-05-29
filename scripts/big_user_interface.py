#! /usr/bin/env python

##@package final_assignment
#This node implements the main user interface 
#
#It's used to manage the output on shell 
# when the bug algoritm is NOT active

# import ros stuff
import rospy
import random
import time
from std_srvs.srv import *


##FUnction for setting a desired position
#
#the six possible positions has been enumerated from 1 to 6
#set the parameters of desired position according to the number that 
#has been extracted. minimun and maximum are 1 and 6 for random target
#but since I use this function also to set the position when is 
#the user to select one of the six, in this case minumum=maximum
#@param minimum minimum of the range
#@param maximum maximum of the range
def random_trgt(minimum, maximum):
    val=random.randint(minimum,maximum)
    if val == 1:
         rospy.set_param("des_pos_x", -4)
         rospy.set_param("des_pos_y", -3)
    if val == 2:
         rospy.set_param("des_pos_x", -4)
         rospy.set_param("des_pos_y", 2)
    if val == 3:
         rospy.set_param("des_pos_x", -4)
         rospy.set_param("des_pos_y", 7)
    if val == 4:
         rospy.set_param("des_pos_x", 5)
         rospy.set_param("des_pos_y", -7)
    if val == 5:
         rospy.set_param("des_pos_x", 5)
         rospy.set_param("des_pos_y", -3)
    if val == 6:
         rospy.set_param("des_pos_x", 5)
         rospy.set_param("des_pos_y", 1)
    
    return []





##Function to assign a new behaviour to the robot
#
#Asks the user to insert a new desired behaviour
#and sets the parameters for the next change in state
def assignment(req):
    #when this service is requested it ask to user to insert a new target
    print("insert a behaviour: ")
    print("1: move randomly")
    print("2: insert a target")
    print("3: fallowing wall")
    print("4: stop")
    print("5: pass to bug algorithm")
    target = int(raw_input('target :'))
    #check the input
    #if target is valid put the inserted value on parameter state
    #put parameter chage to 1 to state that the target has been changed	

    if target==1 or target==2 or target==5:
	   minim=1
	   maxim=6
	   if target == 2:
	   	print("Choose a new position between 6 locations: 1:(-4,-3), 2:(-4,2), 3:(-4,7), 4:(5,-7), 5:(5,-3), 6:(5,1)")
    		minim = int(raw_input('location :'))
		maxim=minim
	   random_trgt(minim,maxim) 
	   rospy.set_param("newstate", target-1)
	   rospy.set_param("change", 1)

    elif target==3 or target==4:
	   rospy.set_param("newstate", target-1)
	   rospy.set_param("change", 1)
    #if input is not valid stop the robot then insert a new target
    else:
           print('not valid input')
           print('robot will be stopped')
           print('please insert a valid target')
    	   rospy.set_param("newstate", 3)
	   rospy.set_param("change", 1)
    return []

##Main function
#
#Initializes the service server for user_interface_assignments
def main():
    #new node
    rospy.init_node('user_interface_assignment')
    #user interface service
    srv = rospy.Service('user_interface_assignment', Empty, assignment)
    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        rate.sleep()


if __name__ == '__main__':
    main()
