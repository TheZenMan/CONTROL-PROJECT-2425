#!/usr/bin/env python3

from . std_msgs import Header
from . geometry_msgs import Pose, PoseWithCovariance, TwistWithCovariance


class Odometry:
    def __init__(self, header=Header(), child_frame_id="",
                 pose=PoseWithCovariance(), twist=TwistWithCovariance):
        self.header = header
        self.child_frame_id = child_frame_id
        self.pose = pose
        self.twist = twist


class MapMetaData:
    def __init__(self, map_load_time=0, resolution=0, width=0,
                 height=0, origin=Pose()):
        self.map_load_time = map_load_time
        self.resolution = resolution
        self.width = width
        self.height = height
        self.origin = origin


class OccupancyGrid:
    def __init__(self, header=Header(), info=MapMetaData(),
                 data=[]):
        self.header = header
        self.info = info
        self.data = data
    