#!/usr/bin/env python3

from . std_msgs import Header


class LaserScan:
    def __init__(self, header=Header(), angle_min=0, angle_max=0,
                 angle_increment=0, time_increment=0, scan_time=0,
                 range_min=0, range_max=0, ranges=[], intensities=[]):
        self.header = header
        self.angle_min = angle_min
        self.angle_max = angle_max
        self.angle_increment = angle_increment
        self.time_increment = time_increment
        self.scan_time = scan_time
        self.range_min = range_min
        self.range_max = range_max
        self.ranges = ranges
        self.intensities = intensities

    def __repr__(self):
        ranges = "["
        length = len(self.ranges)
        for i in range(0, length):
            ranges += str(self.ranges[i])
            if i < length - 1:
                ranges += " "
        ranges += "]"

        intensities = "["
        length = len(self.intensities)
        for i in range(0, length):
            intensities += str(self.intensities[i])
            if i < length - 1:
                intensities += " "
        intensities += "]"

        return "%s, %s, %s, %s, %s, %s, %s, %s, %s, %s" % (
                self.header, self.angle_min, self.angle_max,
                self.angle_increment, self.time_increment, self.scan_time,
                self.range_min, self.range_max, ranges, intensities)

    def __str__(self):
        ranges = "["
        length = len(self.ranges)
        for i in range(0, length):
            ranges += str(self.ranges[i])
            if i < length - 1:
                ranges += " "
        ranges += "]"

        intensities = "["
        length = len(self.intensities)
        for i in range(0, length):
            intensities += str(self.intensities[i])
            if i < length - 1:
                intensities += " "
        intensities += "]"

        return "%s, %s, %s, %s, %s, %s, %s, %s, %s, %s" % (
                self.header, self.angle_min, self.angle_max,
                self.angle_increment, self.time_increment, self.scan_time,
                self.range_min, self.range_max, ranges, intensities)
