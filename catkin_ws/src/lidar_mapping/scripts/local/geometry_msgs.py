#!/usr/bin/env python3

from . std_msgs import Header


class Point:
    def __init__(self, x=0, y=0, z=0):
        self.x = x
        self.y = y
        self.z = z

    def __repr__(self):
        return "%s, %s, %s" % (self.x, self.y, self.z)

    def __str__(self):
        return "%s, %s, %s" % (self.x, self.y, self.z)


class Quaternion:
    def __init__(self, x=0, y=0, z=0, w=0):
        self.x = x
        self.y = y
        self.z = z
        self.w = w

    def __repr__(self):
        return "%s, %s, %s, %s" % (self.x, self.y, self.z, self.w)

    def __str__(self):
        return "%s, %s, %s, %s" % (self.x, self.y, self.z, self.w)


class Vector3:
    def __init__(self, x=0, y=0, z=0):
        self.x = x
        self.y = y
        self.z = z


class Pose:
    def __init__(self, point=Point(), quaternion=Quaternion()):
        self.position = point
        self.orientation = quaternion

    def __repr__(self):
        return "%s, %s" % (self.position, self.orientation)

    def __str__(self):
        return "%s, %s" % (self.position, self.orientation)


class PoseStamped:
    def __init__(self, header=Header(), pose=Pose()):
        self.header = header
        self.pose = pose

    def __repr__(self):
        return "%s, %s" % (self.header, self.pose)

    def __str__(self):
        return "%s, %s" % (self.header, self.pose)


class PoseWithCovariance:
    def __init__(self, pose=Pose(), covariance=[0]*36):
        self.pose = pose
        self.covariance = covariance


class PoseWithCovarianceStamped:
    def __init__(self, header=Header(), pose=PoseWithCovariance()):
        self.header = header
        self.pose = pose


class Twist:
    def __init__(self, linear=Vector3(), angular=Vector3()):
        self.linear = linear
        self.angular = angular


class TwistWithCovariance:
    def __init__(self, twist=Twist(), covariance=[0]*36):
        self.twist = twist
        self.covariance = covariance
