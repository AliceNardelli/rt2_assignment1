#! /usr/bin/env python

import rospy
import time
from rt2_assignment1.srv import Command
import actionlib
import rt2_assignment1.msg
from geometry_msgs.msg import Twist

## Documentation for the main function.
#
#  More details.
#
#
# @var ui_client defines a service client of /user_interface. It taskes as argument the Command service which reaquest is used by user to activate/deactivate robot behaviour. 
# @var client defines an action client  of the go_to_point Action. As argument it takes the message of type GoalReaching action belonging to the rt2_assignment1 package.
# @var pub defines the publsher of the cmd_vel topic. It publishes a Twist to stop the robot. 
# @var rate is a variable to fix the frequency of the spin.
# @var x stores the integer inserted by the user. 

def main():
    # user_interface node inisializsation
    rospy.init_node('user_interface')
    ui_client = rospy.ServiceProxy('/user_interface', Command)
    client = actionlib.SimpleActionClient('go_to_point', rt2_assignment1.msg.GoalReachingAction)
    pub_ = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    time.sleep(10)
    rate = rospy.Rate(20)
    x = int(input("\nPress 1 to start the robot "))
    while not rospy.is_shutdown():
        # if user insert 1 
        if (x == 1):
            # client sents a request to the /user_interface server to activate the robot behaviour
            ui_client("start")
            x = int(input("\nPress 0 to stop the robot "))            
            
        else:
        # Otherwise if user inserts 0 action goal is canceled
            client.cancel_all_goals()
            # Twist msg declared to stop robot in the actual position
            twist_msg = Twist()
            twist_msg.linear.x = 0
            twist_msg.linear.y=0
            twist_msg.angular.z = 0
            #msg published
            pub_.publish(twist_msg)
            #ui_client is now called again to stop the behaviour
            ui_client("stop")
            #asking for another input
            x = int(input("\nPress 1 to start the robot "))
            
if __name__ == '__main__':
    main()


