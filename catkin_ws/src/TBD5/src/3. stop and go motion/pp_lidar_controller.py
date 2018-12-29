#! /usr/bin/env python

import rospy
import math
import numpy as np

from geometry_msgs.msg import Twist
from low_level_interface.msg import lli_ctrl_request
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PointStamped, PoseArray
from sensor_msgs.msg import LaserScan

######################
# TUNABLE PARAMETERS #
######################

k = 0.4  # look forward gain
Lfc = 0.4# look-ahead distance
L = 0.32  # [m] wheel base of vehicle 

target_speed = 40  # [PWM %]

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

    def __init__(self, x, y, yaw, v):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = v


##############
# CONTROLLER #
##############

def pure_pursuit_control(state, cx, cy, pind): #cx, cy are the trajectories to follow


    ind = pind
    tx = cx[ind]  # Select the correct point in the trajectory
    ty = cy[ind]
    alpha = math.atan2(ty - state.y, tx - state.x) - state.yaw

    if state.v < 0:  # To always have a positive velocity
        alpha = math.pi - alpha

    Lf = k * state.v + Lfc

    delta = math.atan2(2.0 * L * math.sin(alpha) / Lf, 1.0)

    return delta, ind


def calc_target_index(state, cx, cy):  # Index for path planning
    """
    Computes the index the pure pursuit needs. Since the trajectories are formed
    as lists containing repetitions of the goal point, this function selects the 
    first element of the list until the distance of the car from the goal point 
    is very small. At that moment the index is incresed in order to stop the pure
    pursuit computation. 
    """

    ind=1
    d=0.05
    dx = state.x - cx[ind]
    dy = state.y - cy[ind]
    Ln = math.sqrt(dx ** 2 + dy ** 2)
    if d < Ln:
	ind = 1
    else:
	ind = len(cx)+1
    
    return ind



#######
# ROS #
#######

rospy.init_node('pp_lidar_controller')
ctrl_pub= rospy.Publisher('/lli/ctrl_request',lli_ctrl_request,queue_size=1)
target_pub = rospy.Publisher('pure_pursuit_target_pose', PointStamped, queue_size=1)

########
# MAIN #
########

traj_x = []
traj_y = []
distance_list=[]

def callback_mocap(odometry_msg):
    global distance_list   # Information from the lidar
    if not len(traj_x) == 0 and not len(distance_list) == 0:  # Computes position of the car only if trajectory and lidar data are available
        x_pos = odometry_msg.pose.pose.position.x
        y_pos = odometry_msg.pose.pose.position.y
        yaw = odometry_msg.pose.pose.orientation.z
	v = odometry_msg.twist.twist.linear.x

        state_m = State(x_pos, y_pos, yaw, v)

        ind = calc_target_index(state_m, traj_x, traj_y)

        if ind < len(traj_x)-1:
            print("### RUNNING TRAJECTORY")
	    dist_len=len(distance_list)
            true_distance_list=[]  # To detect obstacles only a a portion of the lidar information is necessary

	    for i in range (len(distance_list)):
                if i>dist_len/6 and i<5*dist_len/6:  # Select the lidar information the car needs
                    true_distance_list.append(distance_list[i])
            min_dist= min(true_distance_list)   # Among the selected data, find the minimum distance which means the closest obstacle

            if min_dist < 0.60:  # Condition to detect obstacles
                control_request = lli_ctrl_request()
                control_request.velocity = 0  # Stop the car if an obstacle has been detected
                ctrl_pub.publish(control_request)  # publish to control request, but only if near an obstacle
                print("obstacle in way")
            
	    else:
                delta, ind =  pure_pursuit_control(state_m, traj_x, traj_y, ind)
                target_pose = PointStamped()
                target_pose.header.stamp = rospy.Time.now()
                target_pose.header.frame_id = '/qualisys'
                target_pose.point.x = traj_x[ind]
                target_pose.point.y = traj_y[ind]
                target_pub.publish(target_pose)

                target_angle = max(-80, min(delta / (math.pi / 4) * 100, 80))
                control_request = lli_ctrl_request()
                control_request.velocity = target_speed
                control_request.steering = target_angle

        else:
            print("### DONE WITH TRAJECTORY")
            print(len(distance_list))
            control_request = lli_ctrl_request()
            control_request.velocity = 0
            control_request.steering = 0

        ctrl_pub.publish(control_request)


def callback_lidar(scan):
    global distance_list
    if not len(traj_x) == 0: # Conditionn based on the fact that both subscribers do not start same time
        distance_list =scan.ranges
        print('lidar callback')
     

def callback_traj(traj_msg):

	global traj_x
	global traj_y

	traj = traj_msg.poses
	traj_x, traj_y = [], []

	for traj_pt in traj:
		traj_x.append(traj_pt.position.x)
		traj_y.append(traj_pt.position.y)
		

def main():
    mocap_sub = rospy.Subscriber('odometry_body_frame', Odometry, callback_mocap)
    traj_sub = rospy.Subscriber('/nav_traj' + '/SVEA5', PoseArray, callback_traj)


    lidar_sub = rospy.Subscriber('/scan', LaserScan, callback_lidar)

    rospy.spin()

if __name__ == '__main__':
    main()
