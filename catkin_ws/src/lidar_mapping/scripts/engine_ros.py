#!/usr/bin/env python3

"""
    @author: Daniel Duberg (dduberg@kth.se)
"""

# Python standard library
import copy

# ROS
import rospy
import message_filters

# ROS messages
from geometry_msgs.msg import PoseStamped as PoseStampedROS
from sensor_msgs.msg import LaserScan as LaserScanROS
from nav_msgs.msg import OccupancyGrid as OccupancyGridROS
from nav_msgs.msg import Odometry as OdometryROS
from map_msgs.msg import OccupancyGridUpdate as OccupancyGridUpdateROS

# "Local version" of ROS messages
from local.geometry_msgs import PoseStamped
from local.sensor_msgs import LaserScan
from local.nav_msgs import OccupancyGrid
from local.map_msgs import OccupancyGridUpdate

from grid_map import GridMap
from mapping import Mapping


class EngineROS:
    def __init__(self, map_frame_id, map_resolution, map_width, map_height,
                 map_origin_x, map_origin_y, map_origin_yaw, inflate_radius,
                 unknown_space, free_space, c_space, occupied_space, optional=None):
        rospy.init_node('Mapper')
        self.__pose = None
        self.__map = GridMap(map_frame_id, map_resolution, map_width, map_height,
                         map_origin_x, map_origin_y, map_origin_yaw)

        self.__inflated_map = self.__map

        self.__mapping = Mapping(unknown_space, free_space, c_space,
                                 occupied_space, inflate_radius, optional)

        self.__odom_sub = message_filters.Subscriber('odom', OdometryROS)
        self.__scan_sub = message_filters.Subscriber('scan', LaserScanROS)

        self.__ts = message_filters.ApproximateTimeSynchronizer([self.__odom_sub,
                                                     self.__scan_sub], 10, 0.01)
        self.__ts.registerCallback(self.callback)

        self.__map_pub = rospy.Publisher('map', OccupancyGridROS, queue_size=1,
                                         latch=True)
        self.__map_updates_pub = rospy.Publisher("map_updates",
                                                 OccupancyGridUpdateROS,
                                                 queue_size=10)

        self.__map_inflated_pub = rospy.Publisher('inflated_map', OccupancyGridROS, queue_size=1, latch=True)

        self.publish_map()

        rospy.spin()

    def callback(self, odom_ros, scan_ros):
        scan = self.from_ros_scan(scan_ros)
        pose = self.from_ros_odom(odom_ros)

        self.__map, update = self.__mapping.update_map(self.__map, pose, scan)

        if isinstance(update, OccupancyGridUpdate) and len(update.data) != 0:
            self.publish_map_update(update)
            map = copy.deepcopy(self.__map)
            self.__inflated_map = self.__mapping.inflate_map(map)
            self.publish_inflated_map()
        else:
            self.publish_map()

    def publish_map(self):
        # Get ROS occupancy map
        map = self.map_to_message(self.__map)
        self.__map_pub.publish(map)

    def publish_map_update(self, update):
        # Get ROS occupancy map update
        update_ros = self.map_update_to_message(update)
        # Only send out the update
        self.__map_updates_pub.publish(update_ros)

    def publish_inflated_map(self):
        # Get ROS occupancy map
        map = self.map_to_message(self.__inflated_map)
        self.__map_inflated_pub.publish(map)

    def from_ros_scan(self, scan_ros):
        scan = LaserScan()
        scan.header.seq = scan_ros.header.seq
        scan.header.stamp = scan_ros.header.stamp
        scan.header.frame_id = scan_ros.header.frame_id

        scan.angle_min = scan_ros.angle_min
        scan.angle_max = scan_ros.angle_max
        scan.angle_increment = scan_ros.angle_increment
        scan.time_increment = scan_ros.time_increment
        scan.range_min = scan_ros.range_min
        scan.range_max = scan_ros.range_max
        scan.ranges = scan_ros.ranges
        scan.intensities = scan_ros.intensities

        return scan

    def from_ros_odom(self, odom_ros):
        pose_ros = PoseStampedROS()
        pose_ros.header = odom_ros.header
        pose_ros.pose = odom_ros.pose.pose

        pose = PoseStamped()
        pose.header.seq = pose_ros.header.seq
        pose.header.stamp = pose_ros.header.stamp
        pose.header.frame_id = pose_ros.header.frame_id

        pose.pose.position.x = pose_ros.pose.position.x
        pose.pose.position.y = pose_ros.pose.position.y
        pose.pose.position.z = pose_ros.pose.position.z

        pose.pose.orientation.x = pose_ros.pose.orientation.x
        pose.pose.orientation.y = pose_ros.pose.orientation.y
        pose.pose.orientation.z = pose_ros.pose.orientation.z
        pose.pose.orientation.w = pose_ros.pose.orientation.w

        return pose

    def map_to_message(self, map):
        '''
        :type map: Map
        '''
        map = map.to_message()  # OccupancyGrid
        map_ros = OccupancyGridROS()

        # Fill in the header
        map_ros.header.stamp = rospy.Time.now()
        map_ros.header.frame_id = map.header.frame_id

        # Fill in the info
        map_ros.info.resolution = map.info.resolution
        map_ros.info.width = map.info.width
        map_ros.info.height = map.info.height
        map_ros.info.origin = map.info.origin

        # Fill in the map data
        map_ros.data = map.data

        return map_ros

    def map_update_to_message(self, update):
        '''
        :type update: OccupancyGridUpdate
        '''
        update_ros = OccupancyGridUpdateROS()

        # Fill in the header
        update_ros.header.stamp = rospy.Time.now()
        update_ros.header.frame_id = update.header.frame_id

        update_ros.x = update.x
        update_ros.y = update.y
        update_ros.width = update.width
        update_ros.height = update.height
        update_ros.data = update.data

        return update_ros
