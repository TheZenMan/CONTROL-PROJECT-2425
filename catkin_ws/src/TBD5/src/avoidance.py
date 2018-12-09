#! /usr/bin/env python

import rospy
import math
import numpy as np
import matplotlib.pyplot as plt

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
#################
#VISUALIZATION
###############

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
    """
    Given a trajectory, it computes the curvature delta that is necessary to define 
    the steering of the car. The curvature delta is defined each time according to 
    a particular point in the trajectory tx, ty. 
    """

    ind = pind
    tx = cx[ind]   # list of the x coordinates of the goal point
    ty = cy[ind]   # list of the y coordinates of the goal point
    alpha = math.atan2(ty - state.y, tx - state.x) - state.yaw

    if state.v < 0:  # back
        alpha = math.pi - alpha

    Lf = k * state.v + Lfc  #proportional controller for velocity

    delta = math.atan2(2.0 * L * math.sin(alpha) / Lf, 1.0)

    return delta, ind


def calc_target_index(state, cx, cy):
    """
    Defines the index index to select a point in the trajectory. This point is then 
    used by the pure pursuit to compute the curvature delta. 
    """
    ind=1
    d=0.2
    dx = state.x - cx[ind]
    dy = state.y - cy[ind]
    Ln = math.sqrt(dx ** 2 + dy ** 2)  #distance between the position of the car and the goal point
    if d < Ln:
        ind = 1   #since the trajectory is a list that contains repetition of the goal point, if the
                  #distance between the car and the goal point is bigger than a certain treshold, the
                  #first element of the trajectory is taken
    else:
        ind = len(cx)+1   #to stop computing the curvature
    print ind
    print Ln
    return ind

def dist_target(state, cx, cy):
    """
    Computes the distance of the car from the goal point in cx,cy 
    """
    dx = state.x - cx[1]
    dy = state.y - cy[1]
    dist_tg = math.sqrt(dx **2 + dy **2)
    return dist_tg

#######
# ROS #
#######

rospy.init_node('pp_lidar_controller')   #initialization of the node
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
    """
    Callback to obstain information from the Mocap. This function uses the pure pursuit when no obstacles
    are detected, provides an emergy stop and performs obstacle avoidance 
    """   
    global ranges
    global d_min
    if not len(traj_x) == 0 and not len(ranges) == 0:    #unpacking information obtained from the qualisys system
        x_pos = odometry_msg.pose.pose.position.x 
        y_pos = odometry_msg.pose.pose.position.y
        yaw = odometry_msg.pose.pose.orientation.z
        v = odometry_msg.twist.twist.linear.x

        state_m = State(x_pos, y_pos, yaw, v)

        ind = calc_target_index(state_m, traj_x, traj_y)
        min_dist = min(ranges)

        dist_tg = dist_target(state_m, traj_x, traj_y)

        if min_dist < 0.6 and dist_tg > d_min: #if an obstacle is found and the car is far enough from the goal point the velocity is reduced
            angle_list = []
            target_speed = 20
            control_request = lli_ctrl_request()
            control_request.velocity = target_speed
            control_request.steering = 0
            ctrl_pub.publish(control_request)

            for i in range(len(ranges)): # the program might be checking in each increment angle if there is obstacle in the zone
                angle = angle_min + i * increment
                angle_list.append(angle)

            
                if -(90*math.pi/180) <= angle_list[i] <= -(70*math.pi/180):  # the car turn 45[deg] converted in [rad] since LidarScan and car uses [rad] and then multliplied 
                                                                             # by 100 because the car takes in percentage to steer

                    if ranges[i] < 0.2:   #something is detected in the selected range of angles
                        y_steering =180*0.4751131*(abs(angle_list[i]))/(math.pi) - 59.23077   #parameters that give the steering of the car tuned on simulation
                        if angle_list[i] < 0:
                            y_steering = -y_steering   #to ensure the car streers on the right side accordig to the poistion of the detected obstacle
                        control_request = lli_ctrl_request()
                        control_request.velocity = target_speed
                        control_request.steering = (y_steering/1.25 * math.pi / 180) * 100
                        ctrl_pub.publish(control_request)
           

                if -(70*math.pi/180) < angle_list[i] <= -(50*math.pi/180):
                    if ranges[i] < 0.3:
                        y_steering =180*0.4751131*(abs(angle_list[i]))/(math.pi) - 59.23077
                        if angle_list[i] < 0:
                            y_steering = -y_steering
                        control_request = lli_ctrl_request()
                        control_request.velocity = target_speed
                        control_request.steering = (y_steering/1.25 * math.pi / 180) * 100
                        ctrl_pub.publish(control_request)
                      

                if -(50*math.pi/180) < angle_list[i] <= -(30*math.pi/180):
                    if ranges[i] < 0.4:
                        y_steering =180*0.4751131*(abs(angle_list[i]))/(math.pi) - 59.23077
                        if angle_list[i] < 0:
                            y_steering = -y_steering
                        control_request = lli_ctrl_request()
                        control_request.velocity = target_speed
                        control_request.steering = (y_steering/1.25 * math.pi / 180) * 100
                        ctrl_pub.publish(control_request)
                        

                if -(30*math.pi/180) < angle_list[i] <= -(10*math.pi/180):
                    if ranges[i] < 0.5:
                        y_steering =180*0.4751131*(abs(angle_list[i]))/(math.pi) - 59.23077
                        if angle_list[i] < 0:
                            y_steering = -y_steering
                        control_request = lli_ctrl_request()
                        control_request.velocity = target_speed
                        control_request.steering = (y_steering/1.25 * math.pi / 180) * 100
                        ctrl_pub.publish(control_request)
                       

                if -(10*math.pi/180) < angle_list[i] <= (10*math.pi/180):
                    if ranges[i] < 0.6:
                        y_steering =180*0.4751131*(abs(angle_list[i]))/(math.pi) - 59.23077
                        if angle_list[i] < 0:
                            y_steering = -y_steering
                        control_request = lli_ctrl_request()
                        control_request.velocity = target_speed
                        control_request.steering = (y_steering/1.25 * math.pi / 180) * 100
                        ctrl_pub.publish(control_request)
                
                    
                        if ranges[i] < 0.2:    #sort of emergency stop: the car is too close to steer and just stops
                            while min_dist < 0.2:
                                control_request = lli_ctrl_request()
                                control_request.velocity = 0
                                control_request.steering = 0
                                ctrl_pub.publish(control_request)
                                print ("Too close to steer!")

                if (10*math.pi/180) < angle_list[i] <= (30*math.pi/180):
                    if ranges[i] < 0.5:
                        y_steering =180*0.4751131*(abs(angle_list[i]))/(math.pi) - 59.23077
                        control_request = lli_ctrl_request()
                        control_request.velocity = target_speed
                        control_request.steering = (y_steering/1.25 * math.pi / 180) * 100
                        ctrl_pub.publish(control_request)
                       
                if (30*math.pi/180) < angle_list[i] <= (50*math.pi/180):
                    if ranges[i] < 0.4:
                        y_steering =180*0.4751131*(abs(angle_list[i]))/(math.pi) - 59.23077
                        control_request = lli_ctrl_request()
                        control_request.velocity = target_speed
                        control_request.steering = (y_steering/1.25 * math.pi / 180) * 100
                        ctrl_pub.publish(control_request)
                       

                if (50*math.pi/180) < angle_list[i] <= (70*math.pi/180):
                    if ranges[i] < 0.3:
                        y_steering =180*0.4751131*(abs(angle_list[i]))/(math.pi) - 59.23077
                        control_request = lli_ctrl_request()
                        control_request.velocity = target_speed
                        control_request.steering = (y_steering/1.25 * math.pi / 180) * 100
                        ctrl_pub.publish(control_request)
                        

                if (70*math.pi/180) <= angle_list[i] <= (90*math.pi/180):
                    if ranges[i] < 0.2:
                        y_steering =180*0.4751131*(abs(angle_list[i]))/(math.pi) - 59.23077
                        control_request = lli_ctrl_request()
                        control_request.velocity = target_speed
                        control_request.steering = (y_steering/1.25 * math.pi / 180) * 100
                        ctrl_pub.publish(control_request)
                        
                    

        elif min_dist < 0.15:    #emergency stop
            while min_dist < 0.2:
                control_request = lli_ctrl_request()
                control_request.velocity = 0
                control_request.steering = 0
                ctrl_pub.publish(control_request)
                print ("emergency stop!")

        else:
            if ind < len(traj_x)-1:    #since no obstacles are detected, the car follow the pure pursuit while reaching
                                       #the goal point
                print('Running Trajectory')
                target_speed = 30

                ind = calc_target_index(state_m, traj_x, traj_y)
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

            elif min_dist < 0.15:    #emergency stop
                while min_dist < 0.2:
                    control_request = lli_ctrl_request()
                    control_request.velocity = 0
                    control_request.steering = 0
                    ctrl_pub.publish(control_request)
                    print ("emergency stop!")

            else:              
                print("### DONE WITH TRAJECTORY")   

                control_request = lli_ctrl_request()
                control_request.velocity = 0     #the car stops becase it has reached the goal point
                control_request.steering = 0
                ctrl_pub.publish(control_request)


def callback_lidar(scan):   #callback to obtain information from the lidar
    global ranges
    global angle_min
    global increment
    if not len(traj_x) == 0: #Necessary to link the two subscribers
        ranges = scan.ranges
        angle_min = scan.angle_min
        increment = scan.angle_increment

def callback_traj(traj_msg):   #callback to obtain information about the trajectories

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
