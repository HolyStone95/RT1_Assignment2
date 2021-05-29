#! /usr/bin/env python

##@package final_assignment
#This node implements the user interface used by the bug algoritm
#
#It's used to manage the output on shell 
# when the bug algoritm is activated

# import ros stuff
import rospy
from std_srvs.srv import *
import time

##Function to set the new position for bug algoritm
#
#when the service is requested the user has to insert desired position
#@param req request to activate the service
def set_new_pos(req):

    x = float(raw_input('x :'))
    y = float(raw_input('y :'))
    #set parameter user to one, this parameter is used for call the user_interface_assignment only when the target has been reached
    rospy.set_param("user", 1)
    #set the desired position inserted by user between the parameters
    rospy.set_param("des_pos_x", x)
    rospy.set_param("des_pos_y", y)
    print("Thanks! Let's reach the next position")

            
    return []

##Main function
#
#Initialize the server for user_interface service
def main():
    #init node 
    rospy.init_node('user_interface')
    #init service
    srv = rospy.Service('user_interface', Empty, set_new_pos)
    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        rate.sleep()


if __name__ == '__main__':
    main()
