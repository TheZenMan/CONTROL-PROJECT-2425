#!/usr/bin/env python3

from . std_msgs import Header


class OccupancyGridUpdate:
    def __init__(self, header=Header(), x=0, y=0, width=0, height=0, data=[]):
        self.header = header
        self.x = x
        self.y = y
        self.width = width
        self.height = height
        self.data = data