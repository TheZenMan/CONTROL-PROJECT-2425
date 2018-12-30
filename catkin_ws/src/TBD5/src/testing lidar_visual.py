#! /usr/bin/env python

import rospy
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import random

from sensor_msgs.msg import LaserScan

x = np.linspace(-np.pi/2, np.pi/2, 721)
y=np.random.random_integers(0, 5, 721)

plt.ion()
fig = plt.figure()
ax = fig.add_subplot(111)
line1, = ax.plot(x, y, 'b-')

#def callback_lidar(scan):
 #   distance_list = scan.ranges
  #  print(distance_list)
   # print("scan angle min", scan.angle_min)
    #print("scan angle incr", scan.angle_increment)
    #print("scan angle max", scan.angle_max)
    #line1.set_ydata(distance_list)
    #fig.canvas.draw()
#using if loops to get data from scan.ranges?



def callback_lidar(scan):


    ranges = scan.ranges

    angle_min = scan.angle_min
    increment = scan.angle_increment
    line1.set_ydata(scan.ranges)
    fig.canvas.draw()
    #print(scan.ranges[360])
    print(scan.ranges)


rospy.init_node('lidar_visual_node')
rate=rospy.Rate(1)
#rospy.spin()
#plt.show(block=True)
def main():

    lidar_sub = rospy.Subscriber('/scan', LaserScan, callback_lidar)
    plt.show(block=True)
    rate.sleep()

if __name__ == '__main__':
    main()
