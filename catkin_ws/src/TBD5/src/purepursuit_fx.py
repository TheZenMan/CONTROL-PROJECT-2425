#!/usr/bin/env python

import numpy as np
import math
import matplotlib.pyplot as plt
import rospy


from geometry_msgs.msg import Twist

pub = rospy.Publisher('PP_control', Twist, queue_size=10)
rospy.init_node('pure_pursuit', anonymous=True)
rate = rospy.Rate(10) # 10hz
        

  

k = 0.1  # look forward gain
Lfc = 1.0  # look-ahead distance
Kp = 1.0  # speed propotional gain
dt = 0.1  # [s]
L = 2.9  # [m] wheel base of vehicle change according to our car --> length of the car



class State:

    def __init__(self, x=odom_position_x, y=0.0, yaw=0.0, v=0.0):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = v


def update(state, delta):

    state.x = state.x + state.v * math.cos(state.yaw) * dt
    state.y = state.y + state.v * math.sin(state.yaw) * dt
    state.yaw = state.yaw + state.v / L * math.tan(delta) * dt
    state.v = state.v  #constant speed 

    return state


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


def main():
    #  target course
    

    target_speed = 10.0 / 3.6  # [m/s]

    T = 100.0  # max simulation time

    # initial state
    state = State(x=-0.0, y=-3.0, yaw=0.0, v=)

    lastIndex = len(cx) - 1
    time = 0.0
    x = [state.x]
    y = [state.y]
    yaw = [state.yaw]
    v = [state.v]
    t = [0.0]
    target_ind = calc_target_index(state, cx, cy)

    while T >= time and lastIndex > target_ind:
        di, target_ind = pure_pursuit_control(state, cx, cy, target_ind)
        state = update(state, di)

        time = time + dt

        x.append(state.x)
        y.append(state.y)
        yaw.append(state.yaw)
        v.append(state.v)
        t.append(time)


while not rospy.is_shutdown():
        pub.publish(pure_pursuit)
        rate.sleep()

if __name__ == '__main__':
main()

def callback(trajectory_msg): #to be changed

	linear_x = twist_msg.linear.x
	angular_z = twist_msg.angular.z
	control_request = lli_ctrl_request()
	control_request.steering = angular_z
	control_request.velocity = linear_x 
	pub.publish(control_request)


rospy.init_node('topic_subscriber')
sub = rospy.Subscriber('/nav_traj' + self.id, PoseArray, callback)
 
        self.last_time = rospy.get_time()
rospy.spin()

