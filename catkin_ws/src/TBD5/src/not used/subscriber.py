#! /usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from  low_level_interface.msg import lli_ctrl_request

pub = rospy.Publisher('/lli/ctrl_request', lli_ctrl_request, queue_size = 1)

def callback(twist_msg):

    linear_x = twist_msg.linear.x
    angular_z = twist_msg.angular.z
    control_request = lli_ctrl_request()
    control_request.steering = angular_z
    control_request.velocity = linear_x
    pub.publish(control_request)


rospy.init_node('topic_subscriber')
sub = rospy.Subscriber('/key_vel', Twist, callback)

rospy.spin()
