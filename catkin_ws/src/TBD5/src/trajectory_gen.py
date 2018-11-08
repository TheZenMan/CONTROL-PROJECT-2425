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

    LINE_SEGMENT = 0
    SQUARE = 1
    CIRCLE = 2
    ELLIPSE = 3

    def __init__(self, shape):

        self.id = '/SVEA5'
        self.shape = self.str_to_shape(shape)

        self.traj_pub = rospy.Publisher('/nav_traj' + self.id, PoseArray, queue_size=1)
        self.last_time = rospy.get_time()

    def str_to_shape(self, shape_name):
        if   shape_name == "LINE_SEGMENT": return self.LINE_SEGMENT
        elif shape_name == "SQUARE": return self.SQUARE
        elif shape_name == "CIRCLE": return self.CIRCLE
        elif shape_name == "ELLIPSE": return self.ELLIPSE

    def load_traj_msg(self, traj_x, traj_y):

        if not len(traj_x) == len(traj_y):
            raise ValueError("Trajectory not built correctly")

        traj_pose_array = PoseArray()
        for i in range(len(traj_x)):
            pose = Pose()
            pose.position.x = traj_x[i]
            pose.position.y = traj_y[i]
            traj_pose_array.poses.append(pose)
        return traj_pose_array

    def publish_path(self):
        now = rospy.get_time()

        if now-self.last_time > 2:
            self.traj_x, self.traj_y = self.gen_traj()

            msg = self.load_traj_msg(self.traj_x, self.traj_y)
            self.traj_pub.publish(msg)

            self.last_time = now

    def gen_line_segment(self, start_pt, end_pt):
        """
        Generates static line segment from start_pt to end_pt
        start_pt (geometry_msgs/Point), end_pt (geometry_msgs/Point)
        """

        cx, cy = [], []

        # compute additional waypoints
        dst = np.linalg.norm(np.asarray([end_pt.x, end_pt.y]) - \
                             np.asarray([start_pt.x, start_pt.y]))
        angle = np.arctan2(end_pt.y - start_pt.y, end_pt.x - start_pt.x)

        waypts = np.arange(0, dst, self.WAYPOINT_SEPARATION)
        if len(waypts) > 0:
            waypts_x = waypts * np.cos(angle) + start_pt.x * np.ones(waypts.shape)
            waypts_y = waypts * np.sin(angle) + start_pt.y * np.ones(waypts.shape)

            # add waypoints
            cx =  np.append(cx, waypts_x)
            cy = np.append(cy, waypts_y)
        else:
            print("######## WARNING: GENERATED EMPTY TRAJECTORY ########")

        return cx, cy

    def gen_quad(self, corner0, corner1, corner2, corner3, laps=1):

        cx0, cy0 = self.gen_line_segment(corner0, corner1)
        cx1, cy1 = self.gen_line_segment(corner1, corner2)
        cx2, cy2 = self.gen_line_segment(corner2, corner3)
        cx3, cy3 = self.gen_line_segment(corner3, corner0)

        x = np.append(cx0, np.append(cx1, np.append(cx2, cx3)))
        y = np.append(cy0, np.append(cy1, np.append(cy2, cy3)))

        cx, cy = [], []
        for i in range(laps):
            cx = np.append(cx, x)
            cy = np.append(cy, y)

        return cx, cy

    def gen_circle(self, center, radius):

        angle_separation = self.WAYPOINT_SEPARATION / radius
        waypts = np.arange(0, 2 * pi, angle_separation)
        cx = radius * np.cos(waypts)
        cy = radius * np.sin(waypts)

        return cx, cy

    def gen_ellipse(self, center, radius0, radius1):
        print("not implemented yet")
        pass

    def gen_traj(self):

        if self.shape == self.LINE_SEGMENT:
            cx, cy = self.gen_line_segment(Point(-1, -1, 0), Point(1, 1, 0))
        elif self.shape == self.SQUARE:
            cx, cy = self.gen_quad(Point(-1, -1, 0),
                                   Point( 1, -1, 0),
                                   Point( 1,  1, 0),
                                   Point(-1,  1, 0))
        elif self.shape == self.CIRCLE:
            cx, cy = self.gen_circle(Point(), 1)
        elif self.shape == self.ELLIPSE:
            cx, cy = self.gen_ellipse()
        else:
            print("######## WARNING: GIVEN SHAPE NOT HANDLED ########")
            cx, cy = [], []

        return cx, cy

    def listener(self):
        while not rospy.is_shutdown():
            self.publish_path()


def main():

    # initialize ros
    rospy.init_node('trajectory_planner')

    planner_mode = rospy.get_param('/trajectory_planner/planner_mode', "LINE_SEGMENT")

    shape_handler = ShapeHandler(planner_mode)
    shape_handler.listener()

if __name__ == '__main__':
    main()