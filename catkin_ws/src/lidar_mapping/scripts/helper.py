#!/usr/bin/env python3

"""
    @author: Daniel Duberg (dduberg@kth.se)
"""

# Standard Python
from __future__ import print_function
import sys
import time

# Numpy
import numpy as np

# Local version of ROS message files
from local.geometry_msgs import PoseStamped
from local.sensor_msgs import LaserScan

from engine import Engine
try:
    from evaluate import Evaluate
except ImportError:
    pass


# Parameters
map_frame_id = "odom"
map_resolution = 0.025
map_width = 300
map_height = 300
map_origin_x = -map_resolution * map_width / 2
map_origin_y = -map_resolution * map_height / 2
map_origin_yaw = 0

inflate_radius = 5

unknown_space = np.int8(-1)
free_space = np.int8(0)
c_space = np.int8(128)
occupied_space = np.int8(254)

unknown_space_rgb = (128, 128, 128)  # Grey
free_space_rgb = (255, 255, 255)     # White
c_space_rgb = (255, 0, 0)            # Red
occupied_space_rgb = (255, 255, 0)   # Yellow
wrong_rgb = (0, 0, 255)              # Blue


def map_to_rgb(map_data):
    map_data_rgb = [wrong_rgb] * len(map_data)
    # Convert to RGB
    for i in range(0, len(map_data)):
        if map_data[i] == unknown_space:
            map_data_rgb[i] = unknown_space_rgb
        elif map_data[i] == free_space:
            map_data_rgb[i] = free_space_rgb
        elif map_data[i] == c_space:
            map_data_rgb[i] = c_space_rgb
        elif map_data[i] == occupied_space:
            map_data_rgb[i] = occupied_space_rgb
        else:
            # If there is something blue in the image
            # then it is wrong
            map_data_rgb[i] = wrong_rgb

    return map_data_rgb


def get_pose(data):
    # Read PoseStamped data from file
    data = data.split(", ")

    pose = PoseStamped()
    pose.header.seq = int(data[0])
    pose.header.stamp = float(data[1])
    pose.header.frame_id = data[2]

    pose.pose.position.x = float(data[3])
    pose.pose.position.y = float(data[4])
    pose.pose.position.z = float(data[5])

    pose.pose.orientation.x = float(data[6])
    pose.pose.orientation.y = float(data[7])
    pose.pose.orientation.z = float(data[8])
    pose.pose.orientation.w = float(data[9])

    return pose


def get_scan(data):
    # Read LaserScan data from file
    data = data.split(", ")

    scan = LaserScan()
    scan.header.seq = int(data[0])
    scan.header.stamp = float(data[1])
    scan.header.frame_id = data[2]

    scan.angle_min = float(data[3])
    scan.angle_max = float(data[4])
    scan.angle_increment = float(data[5])
    scan.time_increment = float(data[6])
    scan.scan_time = float(data[7])
    scan.range_min = float(data[8])
    scan.range_max = float(data[9])
    scan.ranges = np.array(data[10][1:-1].split(" "))
    scan.ranges = np.asfarray(scan.ranges, float).tolist()
    # We did not save intensities to file, so just set to 0
    scan.intensities = [0.0]*len(scan.ranges)

    return scan


def run_from_file(engine, data, evaluate=None, progress=True):
    evaluation = True
    start_time = time.time()
    i = 0
    num_lines = len(data)
    for line in data:
        i += 1
        if progress:
            if i != num_lines:
                # Clear to the end of line
                sys.stdout.write("\033[K")
                print("Progress {0:2.1%} ({1} of {2}). "
                      .format(float(i) / num_lines, i, num_lines) +
                      "Estimated time remaining: {0} seconds"
                      .format(int((time.time() - start_time) *
                              (num_lines-i) / float(i))), end="\r")
            else:
                # In this case we remove the 'Estimated time remaining'
                # since we are now ~done~.
                # Clear to the end of line
                sys.stdout.write("\033[K")
                print("Progress {0:2.1%} ({1} of {2})"
                      .format(float(i) / num_lines, i, num_lines), end="\r")
            sys.stdout.flush()
        pose_data, scan_data = line.split(";")

        # Remove whitespaces from beginning and end
        pose_data = pose_data.strip()
        scan_data = scan_data.strip()

        # Handle pose
        pose = get_pose(pose_data)

        # Handle scan
        scan = get_scan(scan_data)

        # Call mapper
        engine.callback(pose, scan)

        # Evaluate update
        if evaluate is not None and evaluation:
            evaluation = evaluate.check_update(engine)
    if progress:
        print("\nTotal time taken {0} seconds"
              .format(int(time.time() - start_time)))
    return evaluation
