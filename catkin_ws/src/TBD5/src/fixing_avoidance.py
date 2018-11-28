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

target_speed = 30 # [PWM %]

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
    d=0.2
    dx = state.x - cx[ind]
    dy = state.y - cy[ind]
    Ln = math.sqrt(dx ** 2 + dy ** 2)
    if d < Ln:
        ind = 1
    else:
        ind = len(cx)+1
    print ind
    print Ln
    return ind

def dist_target(state, cx, cy):
    dx = state.x - cx[1]
    dy = state.y - cy[1]
    dist_tg = math.sqrt(dx **2 + dy **2)
    return dist_tg

def two_obstables(ranges, angle_list):
	for i1 in len(ranges):
	for i2 in len(ranges):
            if ranges[i1] < 0.6 and ranges[i2] < 0.6 and -(30*math.pi/180) <= angle_list[i1] <= 0 and 0 <= angle_list[i2] <= (30*math.pi/180):
                a = ranges[i1]
	        a1 = angle_list[i1]
                b = ranges[i2]
                b1 = angle_list[i2]

                w = math.sqrt(math.pow(a,2)+math.pow(b,2)-2*a*b*math.cos(a1-b1))
                w_c = 0.28 # width of car
                    
                if w > w_c:
                    control_request = lli_ctrl_request()
                    control_request.velocity = 25
                    control_request.steering = 0
                    ctrl_pub.publish(control_request)
		else:
		    a_min = min(a1, b1)
	            if a_min <=0:
                        control_request = lli_ctrl_request()
			control_request.velocity = 25
                        control_request.steering = 40*math.pi/180*100
                    	ctrl_pub.publish(control_request)
                            
		    else:
			control_request = lli_ctrl_request()
			control_request.velocity = 25
                        control_request.steering = -40*math.pi/180*100
                    	ctrl_pub.publish(control_request)
         print ("TWO OBSTACLES IN WAY")
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
d_min = 0.1

def callback_mocap(odometry_msg):
    global ranges
    global d_min
    if not len(traj_x) == 0 and not len(ranges) == 0:
        x_pos = odometry_msg.pose.pose.position.x
        y_pos = odometry_msg.pose.pose.position.y
        yaw = odometry_msg.pose.pose.orientation.z
        v = odometry_msg.twist.twist.linear.x

        state_m = State(x_pos, y_pos, yaw, v)
        print("stuck1")
        ind = calc_target_index(state_m, traj_x, traj_y)
        min_dist = min(ranges)

        dist_tg = dist_target(state_m, traj_x, traj_y)

        if min_dist < 0.6 and dist_tg > d_min:
            angle_list = []
            control_request = lli_ctrl_request()
            control_request.velocity = 25
            control_request.steering = 0
            ctrl_pub.publish(control_request)
          

            if min_dist < 0.1:
                control_request = lli_ctrl_request()
                control_request.velocity = 0
                control_request.steering = 0
                ctrl_pub.publish(control_request)
		print ("emergency stop!")
           
            for i in range(len(ranges)): # the program might be checking in each increment angle if there is obstacle in the zone
                angle = angle_min + i * increment
                angle_list.append(angle)

		two_obstacles(ranges, angle_list)
                
                if -(90*math.pi/180) <= angle_list[i] <= -(70*math.pi/180):
                    print("-90 to -70")
                    if ranges[i] < 0.2:
                        print("-90 to -70 and range less than 0.2")
                        control_request = lli_ctrl_request()
                        control_request.velocity = 25
                        control_request.steering = (15*math.pi/180)*100
                        ctrl_pub.publish(control_request)

                if -(70*math.pi/180) < angle_list[i] <= -(50*math.pi/180):
                    print("-70 to -50")
                    if ranges[i] < 0.3:
                        print("-70 to -50 and range less than 0.3")
                        control_request = lli_ctrl_request()
                        control_request.velocity = 25
                        control_request.steering = (25*math.pi/180)*100
                        ctrl_pub.publish(control_request)\

                if -(50*math.pi/180) < angle_list[i] <= -(30*math.pi/180):
                    print("-50 to -30")
                    if ranges[i] < 0.4:
                        print("-50 to -30 and range less than 0.4")
                        control_request = lli_ctrl_request()
                        control_request.velocity = 25
                        control_request.steering = (35*math.pi/180)*100
                        ctrl_pub.publish(control_request)

                if -(30*math.pi/180) < angle_list[i] <= -(10*math.pi/180):
                    print("-30 to -10")
                    if ranges[i] < 0.5:
                        print("-30 to -10 and range less than 0.5")
                        control_request = lli_ctrl_request()
                        control_request.velocity = 25
                        control_request.steering = (45*math.pi/180)*100
                        ctrl_pub.publish(control_request)

                if -(10*math.pi/180) < angle_list[i] <= (10*math.pi/180):
                    print("-10 to 10")
                    if ranges[i] < 0.6:
                        print("-10 to 10 and range less than 0.6")
                        control_request = lli_ctrl_request()
                        control_request.velocity = 25
                        control_request.steering = -(55*math.pi/180)*100
                        ctrl_pub.publish(control_request)

                if (10*math.pi/180) < angle_list[i] <= (30*math.pi/180):
                    print("10 to 30")
                    if ranges[i] < 0.5:
                        print("10 to 30 and range less than 0.5")
                        control_request = lli_ctrl_request()
                        control_request.velocity = 25
                        control_request.steering = -(45*math.pi/180)*100
                        ctrl_pub.publish(control_request)

                if (30*math.pi/180) < angle_list[i] <= (50*math.pi/180):
                    print("30 to 50")
                    if ranges[i] < 0.4:
                        print("30 to 50 and range less than 0.4")
                        control_request = lli_ctrl_request()
                        control_request.velocity = 25
                        control_request.steering = -(35*math.pi/180)*100
                        ctrl_pub.publish(control_request)

                if (50*math.pi/180) < angle_list[i] <= (70*math.pi/180):
                    print("50 to 70")
                    if ranges[i] < 0.3:
                        print("50 to 70 and range less than 0.3")
                        control_request = lli_ctrl_request()
                        control_request.velocity = 25
                        control_request.steering = -(25*math.pi/180)*100
                        ctrl_pub.publish(control_request)

                if (70*math.pi/180) <= angle_list[i] <= (90*math.pi/180):
                    print("70 to 90")
                    if ranges[i] < 0.2:
                        print("70 to 90 and range less than 0.2")
                        control_request = lli_ctrl_request()
                        control_request.velocity = 25
                        control_request.steering = -(15*math.pi/180)*100
                        ctrl_pub.publish(control_request)

        else:
            #control_request = lli_ctrl_request() # think the car stopped when it
            #control_request.velocity = 20        # passed all obstacles, so this
            #control_request.steering = 0         # should bump it a bit forward
            #ctrl_pub.publish(control_request)    # and prevent it to be stopped
            if ind < len(traj_x)-1:
                print('Running Trajectory')

                ind = calc_target_index(state_m, traj_x, traj_y)
                print("stuck2")
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

                ctrl_pub.publish(control_request)

            else:
                print("### DONE WITH TRAJECTORY")

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
