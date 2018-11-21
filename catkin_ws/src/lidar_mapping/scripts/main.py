#!/usr/bin/env python

"""
    @author: Daniel Duberg (dduberg@kth.se)
"""

# Standard Python
from __future__ import print_function
import sys
import time
import os
from os import listdir
from os.path import isfile, join, isdir

# Numpy
import numpy as np

# PIL
try:
    from PIL import Image
except ImportError:
    print("You do not have PIL/Pillow installed")

# Local version of ROS message files
from local.geometry_msgs import PoseStamped
from local.sensor_msgs import LaserScan

from engine import Engine
try:
    from engine_ros import EngineROS
except ImportError:
    pass
from helper import *


def listdir_fullpath(d):
    return [join(d, f) for f in listdir(d)]


def print_help():
    bags_path = os.path.dirname(os.path.realpath(__file__)) + os.sep + ".." + os.sep + "bags" + os.sep + "txt" + os.sep
    bag_files = []
    if isdir(bags_path):
        objects = listdir_fullpath(bags_path)
        for o in objects:
            # Only save the text files
            if isfile(o) and len(o) > 4 and o[-4:] == ".txt":
                bag_files.append(os.path.basename(o[:-4]))

    print("Usage: python main.py [Options] FILE")
    print("")
    print("Where FILE is one of:")
    for file in sorted(bag_files):
        print("  {0}".format(file))
    print("")
    print("If no FILE is given then it will run from the rosbag.")
    print("")
    print("Options:")
    print("  -h                      show this")


def read_sys_args():
    play_file = None
    i = 0
    argv_length = len(sys.argv)
    while i < argv_length-1:
        i += 1
        if str(sys.argv[i]) == "-h":
            print_help()
            exit(0)
        elif str(sys.argv[i][0]) == "-":
            print("Unsupported argument '{0}'. Use '-h' for help."
                  .format(str(sys.argv[i])))
            exit(1)
        else:
            if play_file is not None:
                print("Unsupported argument '{0}'. Use '-h' for help."
                      .format(str(sys.argv[i])))
                exit(1)
            play_file = str(sys.argv[i])
    return play_file

if __name__ == '__main__':
    # Parse input
    play_file = read_sys_args()

    if play_file is None:
        # Playing from rosbag
        print("Playing from rosbag")
        try:
            engine = EngineROS(map_frame_id, map_resolution, map_width,
                               map_height, map_origin_x, map_origin_y,
                               map_origin_yaw, inflate_radius,
                               unknown_space, free_space,
                               c_space, occupied_space)
        except NameError:
            print("You have some problem with ROS")
    else:
        project_path = os.path.dirname(os.path.realpath(__file__)) + os.sep + ".." + os.sep

        # Playing from file
        print("Playing from file")
        engine = Engine(map_frame_id, map_resolution, map_width,
                        map_height, map_origin_x, map_origin_y,
                        map_origin_yaw, inflate_radius, unknown_space,
                        free_space, c_space, occupied_space)

        with open(project_path + "bags" + os.sep + "txt" + os.sep + play_file + ".txt", "r") as file:
            update_correct = run_from_file(engine, file.readlines())

        # Save map to file
        print("Saving map to file")

        map_data = engine.get_map_data()
        inflated_map_data = engine.get_inflated_map_data()

        map_data_rgb = map_to_rgb(map_data)

        map_image = Image.new('RGB', (map_width, map_height))
        map_image.putdata(map_data_rgb)
        map_image = map_image.transpose(Image.FLIP_TOP_BOTTOM)

        if not os.path.exists(project_path + "maps" + os.sep + play_file):
            os.makedirs(project_path + "maps" + os.sep + play_file)

        map_image.save(project_path + "maps" + os.sep + play_file + os.sep + "map.png")

        inflated_map_data_rgb = map_to_rgb(inflated_map_data)
        if map_data_rgb != inflated_map_data_rgb:
            inflated_map_image = Image.new('RGB', (map_width, map_height))
            inflated_map_image.putdata(inflated_map_data_rgb)
            inflated_map_image = inflated_map_image.transpose(Image.FLIP_TOP_BOTTOM)
            
            inflated_map_image.save(project_path + "maps" + os.sep + play_file + os.sep + "inflated_map.png")
