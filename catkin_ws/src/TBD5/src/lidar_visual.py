#! /usr/bin/env python
import rospy
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import random



from sensor_msgs.msg import LaserScan

x = np.linspace(0, 2* np.pi, 100)
y=np.random.random_integers(1, 100, 100)

plt.ion()
fig = plt.figure()
ax = fig.add_subplot(111)
line1, = ax.plot(x, y, 'b-')

def callback(scan):
    distance_list = scan.ranges
    print(distance_list)
    print("scan angle min", scan.angle_min)
    print("scan angle incr", scan.angle_increment)
    print("scan angle max", scan.angle_max)
	line1.set_ydata(distance_list)
	fig.canvas.draw()
#using if loops to get data from scan.ranges?

rospy.init_node('lidar_visual_node')
sub = rospy.Subscriber('\scan', LaserScan, callback)

rospy.spin()