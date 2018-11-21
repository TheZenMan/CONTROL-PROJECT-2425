#!/usr/bin/env python3

"""
    @author: Daniel Duberg (dduberg@kth.se)
"""

# Python standard library
from math import sin, cos

# Numpy
import numpy as np

# "Local version" of ROS messages
from local.geometry_msgs import Point, Pose, Quaternion
from local.nav_msgs import OccupancyGrid


def quaternion_from_euler(roll, pitch, yaw):
    q = [0]*4

    cy = cos(yaw * 0.5)
    sy = sin(yaw * 0.5)
    cr = cos(roll * 0.5)
    sr = sin(roll * 0.5)
    cp = cos(pitch * 0.5)
    sp = sin(pitch * 0.5)

    q[0] = cy * sr * cp - sy * cr * sp
    q[1] = cy * cr * sp + sy * sr * cp
    q[2] = sy * cr * cp - cy * sr * sp
    q[3] = cy * cr * cp + sy * sr * sp

    return q


class GridMap:
    def __init__(self, frame_id="map", resolution=0.05, width=5, height=5,
                 map_origin_x=0, map_origin_y=0, map_origin_yaw=0,
                 default_value=-1):
        self.__frame_id = frame_id
        self.__resolution = resolution
        self.__width = width
        self.__height = height

        q = quaternion_from_euler(0, 0, map_origin_yaw)
        self.__origin = Pose(Point(map_origin_x, map_origin_y, 0),
                             Quaternion(q[0], q[1], q[2], q[3]))

        # default_value = unknown space (-1)
        self.__map = np.full((height, width), default_value, dtype=np.int8)

    def __getitem__(self, pos):
        x, y = pos
        return self.__map[y, x]

    def __setitem__(self, pos, val):
        x, y = pos
        self.__map[int(y), int(x)] = val

    def get_resolution(self):
        return self.__resolution

    def get_width(self):
        return self.__width

    def get_height(self):
        return self.__height

    def get_origin(self):
        return self.__origin

    def to_message(self):
        map = OccupancyGrid()

        # Fill in the header
        map.header.stamp = 0
        map.header.frame_id = self.__frame_id

        # Fill in the info
        map.info.resolution = self.__resolution
        map.info.width = self.__width
        map.info.height = self.__height
        map.info.origin = self.__origin

        # Fill in the map data
        map.data = self.__map.reshape(-1) # (self.__map.size)

        return map
