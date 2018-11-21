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
L = 0.32  # [m] wheel base of

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

def pure_pursuit_control(state, cx, cy, pind): #cx, cy are the trajectories we want to follow


    ind = pind
    tx = cx[ind]
    ty = cy[ind]
    alpha = math.atan2(ty - state.y, tx - state.x) - state.yaw

    if state.v < 0:  # back
        alpha = math.pi - alpha

    Lf = k * state.v + Lfc

    delta = math.atan2(2.0 * L * math.sin(alpha) / Lf, 1.0)

    return delta, ind

#define index for path planning
def calc_target_index(state, cx, cy):
    ind=1
    d=0.05
    dx = state.x - cx[ind]
    dy = state.y - cy[ind]
    Ln = math.sqrt(dx ** 2 + dy ** 2)
    if d < Ln:
	ind = 1
    else:
	ind = len(cx)+1
    print ind
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
ranges=[]

def callback_mocap(odometry_msg):
    global ranges
    if not len(traj_x) == 0 and not len(ranges) == 0:
        x_pos = odometry_msg.pose.pose.position.x
        y_pos = odometry_msg.pose.pose.position.y
        yaw = odometry_msg.pose.pose.orientation.z
	v = odometry_msg.twist.twist.linear.x

        state_m = State(x_pos, y_pos, yaw, v)

        ind = calc_target_index(state_m, traj_x, traj_y)

        if ind < len(traj_x)-1:
	    angle_list = []
	    # if scan.ranges < 1:
	    for i in range(len(ranges)): # the program might be checking in each increment angle if there is obstacle in the zone
		angle = angle_min + i * increment
	   	angle_list.append(angle)
            
                control_request = lli_ctrl_request()
		control_request.velocity = 25

		    # Want the car to turn 45[deg] so here it calculates [rad] since LidarScan and car uses [rad] then mutliply by 100 because the car takes in percentage to steer
		if -(90*math.pi/180) <= angle_list[i] <= -(70*math.pi/180):
		    if ranges[i] < 0.2:  
    			control_request.steering = (15*math.py/180)*100
			ctrl_pub.publish(control_request)
		    #elif 0.2 < ranges[i] < 0.4:
		         #    control_request.steering = 0
		    #else:
		 	 #   control_request.steering = target_angle

		if -(69*math.pi/180) <= angle_list[i] <= -(50*math.pi/180):
		    if ranges[i] < 0.4:
			control_request.steering = (25*math.py/180)*100
			ctrl_pub.publish(control_request)
		    #else:
			 #   control_request.steering = target_angle

		if -(49*math.pi/180) <= angle_list[i] <= (30*math.pi/180):
		    if ranges[i] < 0.6:
			control_request.steering = (35*math.py/180)*100
			ctrl_pub.publish(control_request)
		    #else:
	 		#   control_request.steering = target _angle

		if -(29*math.pi/180) <= angle_list[i] <= -(10*math.pi/180):
		    if ranges[i] < 0.8:
			control_request.steering = (45*math.py/180)*100
			ctrl_pub.publish(control_request)
		    #else:
		 	#    control_request.steering = -(15*math.py/180)*100

		if -(9*math.pi/180) < angle_list[i] <= (10*math.py/180):
		    if ranges[i] < 1:
			control_request.steering = -(55*math.py/180)*100
			ctrl_pub.publish(control_request)
		    #else:
			#   control_request.steering = target_angle

		if (11*math.pi/180) <= angle_list[i] <= (30*math.pi/180):
		    if ranges[i] < 0.8:  
    			control_request.steering = -(45*math.py/180)*100 
			ctrl_pub.publish(control_request)
		    #else:
		 	#   control_request.steering = target_angle

		if (31*math.pi/180) <= angle_list[i] <= (50*math.pi/180):
		    if ranges[i] < 0.6:
			control_request.steering = -(35*math.py/180)*100
			ctrl_pub.publish(control_request)
		    #else:
			#   control_request.steering = target_angle

		if (51*math.pi/180) <= angle_list[i] <= (70*math.pi/180):
		    if ranges[i] < 0.4:
			control_request.steering = -(25*math.py/180)*100
			ctrl_pub.publish(control_request)
		    #else:
	 		#   control_request.steering = target _angle

		if (71*math.pi/180) <= angle_list[i] <= (90*math.pi/180):
		    if ranges[i] < 0.2:
			control_request.steering = -(15*math.py/180)*100
			ctrl_pub.publish(control_request)
		    #elif 0.2 < ranges[i] < 0.4:
		     #   control_request.steering = 0
		    #else:
		 	#   control_request.steering = (15*math.py/180)*100
			
		    #ctrl_pub.publish(control_request)
	
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
        print(len(ranges))
        control_request = lli_ctrl_request()
        control_request.velocity = 0
        control_request.steering = 0

    ctrl_pub.publish(control_request)

def callback_lidar(scan):
    global ranges
    global angle_min
    global increment
    if not len(traj_x) == 0: #both subscribers dont start same time
        ranges = scan.ranges
	angle_min = scan.angle_min
	increment = scan.angle_increment

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
