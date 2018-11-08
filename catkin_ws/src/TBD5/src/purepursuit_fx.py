#!/usr/bin/env python

import rospy
import math
import numpy as np

from geometry_msgs.msg import Twist


######################
# TUNABLE PARAMETERS #
######################

k = 0.1  # look forward gain
Lfc = 1.0  # look-ahead distance
Kp = 1.0  # speed propotional gain
L = 0.32  # [m] wheel base of vehicle change according to our car --> length of the car

target_speed = 10.0 / 3.6  # [m/s]

####################
# GLOBAL VARIABLES #
####################

# initialized state
x = []
y = []
yaw = []
v = []


#####################
# CLASS DEFINITIONS #
#####################

class State:

    def __init__(self, x=0.0, y=0.0, yaw=0.0, v=0.0):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = v


##############
# CONTROLLER #
##############

def pure_pursuit_control(state, cx, cy, pind): #cx, cy are the trajectories we want to follow

    ind = calc_target_index(state, cx, cy)

    if pind >= ind:
        ind = pind

    if ind < len(cx):
        tx = cx[ind]
        ty = cy[ind]
    else:
        tx = cx[-1]
        ty = cy[-1]
        ind = len(cx) - 1

    alpha = math.atan2(ty - state.y, tx - state.x) - state.yaw

    if state.v < 0:  # back
        alpha = math.pi - alpha

    Lf = k * state.v + Lfc

    delta = math.atan2(2.0 * L * math.sin(alpha) / Lf, 1.0)

    return delta, ind


def calc_target_index(state, cx, cy):

    # search nearest point index
    dx = [state.x - icx for icx in cx]
    dy = [state.y - icy for icy in cy]
    d = [abs(math.sqrt(idx ** 2 + idy ** 2)) for (idx, idy) in zip(dx, dy)]
    ind = d.index(min(d))
    Ln = 0.0

    Lf = k * state.v + Lfc

    # search look ahead target point index
    while Lf > Ln and (ind + 1) < len(cx):
        dx = cx[ind + 1] - cx[ind]
        dy = cx[ind + 1] - cx[ind]
        Ln += math.sqrt(dx ** 2 + dy ** 2)
        ind += 1

    return ind


#######
# ROS #
#######

# TODO: put these three lines in the right place
# state = State()
# target_ind = calc_target_index(state, cx, cy)
# di, target_ind = pure_pursuit_control(state, cx, cy, target_ind)

pub = rospy.Publisher('PP_control', Twist, queue_size=10)
rospy.init_node('pure_pursuit', anonymous=True)
rate = rospy.Rate(30) # 30 [Hz]

def traj_callback(trajectory_msg): #to be changed

        linear_x = twist_msg.linear.x
        angular_z = twist_msg.angular.z
        control_request = lli_ctrl_request()
        control_request.steering = angular_z
        control_request.velocity = linear_x
        pub.publish(control_request)


########
# MAIN #
########

def main():

    rospy.init_node('pure_pursuit_controller')

    traj_sub = rospy.Subscriber('/nav_traj' + self.id, PoseArray, traj_callback)

    rospy.spin()


if __name__ == '__main__':
    main()
