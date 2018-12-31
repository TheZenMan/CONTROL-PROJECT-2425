#! /usr/bin/env python

import rospy
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import random

from sensor_msgs.msg import LaserScan

x = np.linspace(-np.pi/2, np.pi/2, 721)  # Creates the axes for the plot
y=np.random.random_integers(0, 5, 721)

plt.ion()
fig = plt.figure()
ax = fig.add_subplot(111)
line1, = ax.plot(x, y, 'b-')


def callback_lidar(scan):  # Takes the information from the Lidar


    ranges = scan.ranges
    angle_min = scan.angle_min
    increment = scan.angle_increment

    line1.set_ydata(scan.ranges)
    fig.canvas.draw()
    print(scan.ranges)


rospy.init_node('lidar_visual_node')
rate=rospy.Rate(1)


def main():
    lidar_sub = rospy.Subscriber('/scan', LaserScan, callback_lidar)
    plt.show(block=True)
    rate.sleep()

if __name__ == '__main__':
    main()
