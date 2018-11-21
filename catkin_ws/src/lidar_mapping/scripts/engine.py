#!/usr/bin/env python3

"""
    @author: Daniel Duberg (dduberg@kth.se)
"""

# Python standard library
import copy

# Numpy
import numpy as np

# "Local version" of ROS messages
from local.geometry_msgs import PoseStamped
from local.sensor_msgs import LaserScan
from local.map_msgs import OccupancyGridUpdate

from grid_map import GridMap
from mapping import Mapping


class Engine():
    def __init__(self, map_frame_id, map_resolution, map_width, map_height,
                 map_origin_x, map_origin_y, map_origin_yaw, inflate_radius,
                 unknown_space, free_space, c_space, occupied_space,
                 optional=None):
        self.__pose = None
        self.__map = GridMap(map_frame_id, map_resolution, map_width,
                             map_height, map_origin_x, map_origin_y,
                             map_origin_yaw, unknown_space)

        self.__inflated_map = self.__map

        self.__mapping = Mapping(unknown_space, free_space, c_space,
                                 occupied_space, inflate_radius, optional)

        self.__update = None

        self.__correct_inflated_map = True

    def callback(self, pose, scan):
        self.__correct_inflated_map = False
        self.__map, self.__update = self.__mapping.update_map(self.__map, pose,
                                                              scan)

    def get_map_data(self):
        return self.__map.to_message().data.tolist()

    def get_inflated_map_data(self):
        if not self.__correct_inflated_map:
            # Lazy-evaluation of the inflated map
            grid_map = copy.deepcopy(self.__map)
            self.__inflated_map = self.__mapping.inflate_map(grid_map)

        return self.__inflated_map.to_message().data.tolist()

    def get_update(self):
        return self.__update

    def get_map(self):
        return self.__map

    def get_inflated_map(self):
        if not self.__correct_inflated_map:
            # Lazy-evaluation of the inflated map
            grid_map = copy.deepcopy(self.__map)
            self.__inflated_map = self.__mapping.inflate_map(grid_map)

        return self.__inflated_map
