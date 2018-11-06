#! /usr/bin/env python

import rospy
from  low_level_interface.msg import lli_ctrl_request
rospy.init_node('cs_publisher')
pub = rospy.Publisher('/lli/ctrl_request',lli_ctrl_request,queue_size=1)
rate = rospy.Rate(50)
control_request = lli_ctrl_request()
control_request.velocity = 30
theta=50
while not rospy.is_shutdown
	control_request.steering = theta
	pub.publish(control_request)
	rate.sleep()



