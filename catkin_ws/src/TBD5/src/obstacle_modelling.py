#!/usr/bin/env python

import sys
import os
import rospy
import numpy as np
from math import pi, sin, cos, sqrt, atan2, radians, degrees

from geometry_msgs.msg import Point
from geometry_msgs.msg import Pose, PoseArray, PoseWithCovarianceStamped


###########################
## SHAPE HANDLING OBJECT ##
###########################

class ShapeHandler:

    WAYPOINT_SEPARATION = 0.1
    SQUARE = 1


def __init__(self, shape):

     self.id = '/SVEA5'
     self.shape = self.str_to_shape(shape)

     self.traj_pub = rospy.Publisher('/nav_traj' + self.id, PoseArray, queue_size=1)
     self.last_time = rospy.get_time()

def str_to_shape(self, shape_name):
        
     shape_name == "SQUARE": return self.SQUARE
  
corner1 = corner0 + 0.45
corner2 = corner0 + 0.95
corner3 = corner1 + 0.95

def gen_quad(self, corner0, corner1, corner2, corner3, laps=1):

        x0, y0 = self.gen_line_segment(corner0, corner1)
        x1, y1 = self.gen_line_segment(corner1, corner2)
        x2, y2 = self.gen_line_segment(corner2, corner3)
        x3, y3 = self.gen_line_segment(corner3, corner0)
        x4, y4 = self.gen_line_segment(corner0, corner1)

        x = np.append(x0, np.append(x1, np.append(x2, np.append(x3, x4))))
        y = np.append(y0, np.append(y1, np.append(y2, np.append(y3, y4))))

        cx, cy = [], []
        for i in range(laps):
            cx = np.append(cx, x)
            cy = np.append(cy, y)

        return cx, cy

def gen_traj(self):

	for i in range(len(ranges)): # the program might be checking in each increment angle if there is obstacle in the zone
                angle = angle_min + i * increment
                angle_list.append(angle)

        self.shape == self.SQUARE:
            cx, cy = self.gen_quad(Point(1, -1, 0), #TRANSFORM POSITION OF THE IDENTIFIED OBSTACLE TO DEFINE THE REGION OF THE OBSTACLE
                                   Point( 1, -1, 0),
                                   Point( 1,  1, 0),
                                   Point(-1,  1, 0))
        elif self.shape == self.CIRCLE:
            cx, cy = self.gen_circle(Point(), 1)
        elif self.shape == self.ELLIPSE:
            cx, cy = self.gen_ellipse()
        elif self.shape == self.POINT:
            cx, cy = self.gen_point(Point(-1,1,0))

        else:
            print("######## WARNING: GIVEN SHAPE NOT HANDLED ########")
            cx, cy = [], []

        return cx, cy

    def listener(self):
        while not rospy.is_shutdown():
            self.publish_path()



def callback_lidar(scan):
    global ranges
    global angle_min
    global increment
    ranges = scan.ranges
    angle_min = scan.angle_min
    increment = scan.angle_increment


def main():

    # initialize ros
    rospy.init_node('osbstacles_modelling')

    planner_mode = rospy.get_param('/trajectory_planner/planner_mode', "SQUARE")


    shape_handler = ShapeHandler(planner_mode)
    

if __name__ == '__main__':
    main()


