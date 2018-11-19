#!/usr/bin/env python

import rospy
import math
import numpy as np

from geometry_msgs.msg import Twist
from  low_level_interface.msg import lli_ctrl_request
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PointStamped, PoseArray


######################
# TUNABLE PARAMETERS #
######################

k = 0.4  # look forward gain
Lfc = 0.4# look-ahead distance
#Kp = 0.7  # speed propotional gain
L = 0.32  # [m] wheel base of
#vehicle change according to our car --> length of the car

target_speed = 35  # [PWM %]

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
    ind = 1
   #ind = calc_target_index(state, cx, cy)

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

#def calc_target_index(state, cx, cy):  #uncomment for path following

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
        dy = cy[ind + 1] - cy[ind]
        Ln += math.sqrt(dx ** 2 + dy ** 2)
        ind += 1
        print ind
    return ind

    #define index for path planning
#def calc_target_index(state, cx, cy):

    dx = [state.x - icx for icx in cx]
    dy = [state.y - icy for icy in cy]
    d = [abs(math.sqrt(idx ** 2 + idy ** 2)) for (idx, idy) in zip(dx, dy)]
    ind = 1
    Ln = 0.0

    Lf = k * state.v + Lfc

    while Lf > Ln and (ind + 1) < len(cx):
        dx = cx[ind + 1] - cx[ind]
        dy = cy[ind + 1] - cy[ind]
        Ln += math.sqrt(dx ** 2 + dy ** 2)
        ind +=1
        print ind

    return ind

#######
# ROS #
#######

# TODO: put these three lines in the right place
# state = State()
# target_ind = calc_target_index(state, cx, cy)
# di, target_ind = pure_pursuit_control(state, cx, cy, target_ind)


#pub= rospy.Publisher('/lli/ctrl_request',lli_ctrl_request,queue_size=1)

rospy.init_node('pure_pursuit_controller')
ctrl_pub= rospy.Publisher('/lli/ctrl_request',lli_ctrl_request,queue_size=1)
target_pub = rospy.Publisher('pure_pursuit_target_pose', PointStamped, queue_size=1)

#print("test1")

########
# MAIN #
########

traj_x = []
traj_y = []
# pind = 0

def callback_mocap(odometry_msg): # ask Frank
    # global pind
    #print("mocap")
    if not len(traj_x) == 0:
        #while pind<len(traj_x):
        x_pos = odometry_msg.pose.pose.position.x
        y_pos = odometry_msg.pose.pose.position.y
	#print(x_pos)
        yaw = odometry_msg.pose.pose.orientation.z
	v = odometry_msg.twist.twist.linear.x
	#v=30

        state_m = State(x_pos, y_pos, yaw, v)

        #ind = calc_target_index(state_m, traj_x, traj_y)
        ind = 1

        if ind < len(traj_x)-1:
            print("### RUNNING TRAJECTORY")

            delta, ind =  pure_pursuit_control(state_m, traj_x, traj_y, ind)
            # pind = ind

            target_pose = PointStamped()
            target_pose.header.stamp = rospy.Time.now()
            target_pose.header.frame_id = 'qualisys'
            # target_pose.point.x = traj_x[pind]
            # target_pose.point.y = traj_y[pind]
            target_pose.point.x = traj_x[ind]
            target_pose.point.y = traj_y[ind]
            target_pub.publish(target_pose)

            target_angle = max(-80, min(delta / (math.pi / 4) * 100, 80))
            #print(target_angle)
            control_request = lli_ctrl_request()
            control_request.velocity = target_speed
            control_request.steering = target_angle

        else:
            print("### DONE WITH TRAJECTORY")

            control_request = lli_ctrl_request()
            control_request.velocity = 0
            control_request.steering = 0

        ctrl_pub.publish(control_request)

def callback_traj(traj_msg):
	#print("traj")
    #traj_x = traj_msg.poses.position.x # Ask Frank
    #traj_y = traj_msg.poses.position.y

	global traj_x
	global traj_y

	traj = traj_msg.poses
	traj_x, traj_y = [], []

	for traj_pt in traj:
		traj_x.append(traj_pt.position.x)
		traj_y.append(traj_pt.position.y)
		#print(traj_pt.position.x)

def main():
   # rospy.init_node('pure_pursuit_controller')
   # pub= rospy.Publisher('/lli/ctrl_request',lli_ctrl_request,queue_size=1)
   #rate = rospy.Rate(50) # 30 [Hz]
    #print("test_main")
    mocap_sub = rospy.Subscriber('odometry_body_frame', Odometry, callback_mocap)
    traj_sub = rospy.Subscriber('/nav_traj' + '/SVEA5', PoseArray, callback_traj)
    #rate = rospy.Rate(50) # 30 [Hz]

    rospy.spin()

if __name__ == '__main__':
    main()

