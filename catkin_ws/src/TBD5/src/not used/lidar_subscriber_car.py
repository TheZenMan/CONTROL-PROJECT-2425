#! /usr/bin/env python

import rospy

from sensor_msgs.msg import LaserScan


def callback(scan):
    distance_list = scan.ranges
    print(distance_list)
    print("scan angle min", scan.angle_min)
    print("scan angle incr", scan.angle_increment)
    print("scan angle max", scan.angle_max)



rospy.init_node('laser_subscriber')
sub = rospy.Subscriber('\scan', LaserScan, callback)

rospy.spin()