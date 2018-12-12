#!/usr/bin/env python3

"""
    # Frank Jiang
    # frankji@kth.se
"""

import rospy

# Python standard library
from math import cos, sin, atan2, fabs, sqrt
from math import pi, ceil

# Numpy
import numpy as np

# "Local version" of ROS messages
from local.geometry_msgs import PoseStamped, Quaternion
from local.sensor_msgs import LaserScan
from local.map_msgs import OccupancyGridUpdate

from std_msgs.msg import Int8MultiArray

from grid_map import GridMap

import pdb

class Mapping:
    def __init__(self, unknown_space, free_space, c_space, occupied_space,
                 radius, optional=None):
        self.unknown_space = unknown_space
        self.free_space = free_space
        self.c_space = c_space
        self.occupied_space = occupied_space
        self.allowed_values_in_map = {"self.unknown_space": self.unknown_space,
                                      "self.free_space": self.free_space,
                                      "self.c_space": self.c_space,
                                      "self.occupied_space": self.occupied_space}
        self.radius = radius
        self.__optional = optional

        # self.grid_publisher = rospy.Publisher('/persistent_grid_map', Int8MultiArray, queue_size = 1)

    def get_yaw(self, q):
        """Returns the Euler yaw from a quaternion.
        :type q: Quaternion
        """
        return atan2(2 * (q.w * q.z + q.x * q.y),
                     1 - 2 * (q.y * q.y + q.z * q.z))

    def raytrace(self, start, end):
        """Returns all cells in the grid map that has been traversed
        from start to end, including start and excluding end.
        start = (x, y) grid map index
        end = (x, y) grid map index
        """
        (start_x, start_y) = start
        (end_x, end_y) = end
        x = start_x
        y = start_y
        (dx, dy) = (fabs(end_x - start_x), fabs(end_y - start_y))
        n = dx + dy
        x_inc = 1
        if end_x <= start_x:
            x_inc = -1
        y_inc = 1
        if end_y <= start_y:
            y_inc = -1
        error = dx - dy
        dx *= 2
        dy *= 2

        traversed = []
        for i in range(0, int(n)):
            traversed.append((int(x), int(y)))

            if error > 0:
                x += x_inc
                error -= dy
            else:
                if error == 0:
                    traversed.append((int(x + x_inc), int(y)))
                y += y_inc
                error += dx

        return traversed

    def add_to_map(self, grid_map, x, y, value):
        """Adds value to index (x, y) in grid_map if index is in bounds.
        Returns weather (x, y) is inside grid_map or not.
        """
        if value not in self.allowed_values_in_map.values():
            raise Exception("{0} is not an allowed value to be added to the map. "
                            .format(value) + "Allowed values are: {0}. "
                            .format(self.allowed_values_in_map.keys()) +
                            "Which can be found in the '__init__' function.")

        if self.is_in_bounds(grid_map, x, y):
            grid_map[x, y] = value
            return True
        return False

    def is_in_bounds(self, grid_map, x, y):
        """Returns weather (x, y) is inside grid_map or not."""
        if x >= 0 and x < grid_map.get_width():
            if y >= 0 and y < grid_map.get_height():
                return True
        return False

    def update_map(self, grid_map, pose, scan):
        """Updates the grid_map with the data from the laser scan and the pose.

        For E:
            Update the grid_map with self.occupied_space.

            Return the updated grid_map.

            You should use:
                self.occupied_space  # For occupied space

                You can use the function add_to_map to be sure that you add
                values correctly to the map.

                You can use the function is_in_bounds to check if a coordinate
                is inside the map.

        For C:
            Update the grid_map with self.occupied_space and self.free_space. Use
            the raytracing function found in this file to calculate free space.

            You should also fill in the update (OccupancyGridUpdate()) found at
            the bottom of this function. It should contain only the rectangle area
            of the grid_map which has been updated.

            Return both the updated grid_map and the update.

            You should use:
                self.occupied_space  # For occupied space
                self.free_space      # For free space

                To calculate the free space you should use the raytracing function
                found in this file.

                You can use the function add_to_map to be sure that you add
                values correctly to the map.

                You can use the function is_in_bounds to check if a coordinate
                is inside the map.

        :type grid_map: GridMap
        :type pose: PoseStamped
        :type scan: LaserScan
        """

        # Current yaw of the robot
        robot_yaw = self.get_yaw(pose.pose.orientation)
        # The origin of the map [m, m, rad]. This is the real-world pose of the
        # cell (0,0) in the map.
        origin_pose = grid_map.get_origin()
        origin_yaw = self.get_yaw(origin_pose.orientation)
        origin = [origin_pose.position.x, origin_pose.position.y,
                  origin_yaw]
        # The map resolution [m/cell]
        resolution = grid_map.get_resolution()


        """
        Fill in your solution here
        """

        def scan_to_rel_poses(scan):
            poses = []

            #unpack LaserScan msg
            angle_min, angle_max = scan.angle_min, scan.angle_max
            angle_increment = scan.angle_increment
            ranges = scan.ranges

            curr_angle = angle_min
            for dist in ranges:
                if dist > scan.range_min and dist < scan.range_max:
                    rel_x, rel_y = dist * cos(curr_angle), dist * sin(curr_angle)
                    poses.append([rel_x, rel_y])
                curr_angle += angle_increment

            return np.array(poses)

        def rotate(origin, pt, angle):
            origin_x, origin_y = origin[0], origin[1]
            pt_x, pt_y = pt[0], pt[1]

            dist_x = pt_x - origin_x
            dist_y = pt_y - origin_y

            rot_x = origin_x + cos(angle) * (dist_x) - \
                               sin(angle) * (dist_y)
            rot_y = origin_y + sin(angle) * (dist_x) + \
                               cos(angle) * (dist_y)
            return [rot_x, rot_y]

        robot_x, robot_y = pose.pose.position.x, pose.pose.position.y

        # convert scans to relative positions
        scan_rel_poses = scan_to_rel_poses(scan)
        # rotate relative positions by robot heading (yaw)
        scan_poses = np.array([rotate([0, 0], rel_pose, robot_yaw)
                               for rel_pose in scan_rel_poses])
        # translate to robot position within coordinate frame
        scan_poses[:, 0] += robot_x * np.ones(scan_poses.shape[0])
        scan_poses[:, 1] += robot_y * np.ones(scan_poses.shape[0])

        # transform for coordinate frame position and rotation
        scan_poses[:, 0] -= origin[0] * np.ones(scan_poses.shape[0])
        scan_poses[:, 1] -= origin[1] * np.ones(scan_poses.shape[0])

        robot_cell_x = (robot_x-origin[0])/resolution #change if pose not based on origin
        robot_cell_y = (robot_y-origin[1])/resolution

        updates = []
        laser_pts = []
        # gather laser pts
        for i in range(scan_poses.shape[0]):
            curr_pose = scan_poses[i, :]

            curr_pose[0] = curr_pose[0]/resolution
            curr_pose[1] = curr_pose[1]/resolution

            curr_pose[0] = int(curr_pose[0])
            curr_pose[1] = int(curr_pose[1])
            laser_pts.append(curr_pose)

        for laser_pt in laser_pts:
            # compute free space grid cells
            start = (int(robot_cell_x), int(robot_cell_y))
            end = (laser_pt[0], laser_pt[1])
            free_spaces = self.raytrace(start, end)

            # add free spaces, but keep c spaces
            for free_space in free_spaces:
                if free_space[0] < grid_map.get_width and free_space[1] < grid_map.get_height:
                    if not grid_map[free_space[0], free_space[1]] == self.c_space:
                        self.add_to_map(grid_map, free_space[0], free_space[1],
                                        self.free_space)

            updates += free_spaces

        # grid_msg = Int8MultiArray()
        # grid_msg.data = np.asarray(updates, dtype='int8')
        # self.grid_publisher.publish(grid_msg)

        for laser_pt in laser_pts:
            self.add_to_map(grid_map, laser_pt[0], laser_pt[1],
                            self.occupied_space)
            updates.append(laser_pt)

        # find update parameters
        updates = np.array(updates)
        min_update_x, min_update_y = int(min(updates[:, 0])-1), int(min(updates[:, 1])-1)
        max_update_x, max_update_y = int(max(updates[:, 0])+1), int(max(updates[:, 1])+1)
        update_width = int(max_update_x - min_update_x + 1)
        update_height = int(max_update_y - min_update_y + 1)

        # extract data for update from grid_map
        update_data = []
        for j in range(update_height):
            for i in range(update_width):
                update_data.append(grid_map[min_update_x + i, min_update_y + j])

        # Only get the part that has been updated
        update = OccupancyGridUpdate()
        # The minimum x index in 'grid_map' that has been updated
        update.x = min_update_x
        # The minimum y index in 'grid_map' that has been updated
        update.y = min_update_y
        # Maximum x index - minimum x index + 1
        update.width = update_width
        # Maximum y index - minimum y index + 1
        update.height = update_height
        # The map data inside the rectangle, in row-major order.
        update.data = update_data

        # Return the updated map together with only the
        # part of the map that has been updated
        return grid_map, update

    def inflate_map(self, grid_map):
        """For C only!
        Inflate the map with self.c_space assuming the robot
        has a radius of self.radius.

        Returns the inflated grid_map.

        Inflating the grid_map means that for each self.occupied_space
        you calculate and fill in self.c_space. Make sure to not overwrite
        something that you do not want to.


        You should use:
            self.c_space  # For C space (inflated space).
            self.radius   # To know how much to inflate.

            You can use the function add_to_map to be sure that you add
            values correctly to the map.

            You can use the function is_in_bounds to check if a coordinate
            is inside the map.

        :type grid_map: GridMap
        """

        def set_circle_c_space(i, j):

            for ii in range(i-self.radius-1, i+self.radius+1):
                for jj in range(j-self.radius-1, j+self.radius+1):
                    curr_dist = sqrt((i-ii)**2 + (j-jj)**2)
                    if curr_dist <= self.radius:
                        if self.is_in_bounds(grid_map, ii, jj) and \
                            not grid_map[ii, jj] == self.occupied_space:

                            self.add_to_map(grid_map, ii, jj,
                                            self.c_space)

        # width, height = grid_map.get_width(), grid_map.get_height()
        # for i in range(width):
            # for j in range(height):
                # if grid_map[i, j] == self.occupied_space:
                    # set_circle_c_space(i, j)

        # Return the inflated map
        return grid_map

