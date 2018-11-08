#! /usr/bin/env python

import rospy
from  low_level_interface.msg import lli_ctrl_request
pub = rospy.Publisher('/lli/ctrl_request',llictrl_request,queue_size=1)

control_request = lli_ctrl_request()
control_request.velocity = 30
theta=0
for theta in range(0,100)
	control_request.steering = theta
	pub.publish(control_request)
	theta +=10



