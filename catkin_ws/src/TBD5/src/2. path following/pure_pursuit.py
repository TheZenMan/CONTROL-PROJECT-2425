#!/usr/bin/env python

import rospy # Import Python ros as well as math
import math

from low_level_interface.msg import lli_ctrl_request
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PointStamped, PoseArray # Imports msg used in file


######################
# TUNABLE PARAMETERS #
######################

k = 0.4  # look forward gain
Lfc = 0.4# look-ahead distance
L = 0.32  # [m] wheel base of
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
# Pure pursuit function where cx, cy are the trajectories to follow
def pure_pursuit_control(state, cx, cy, pind):

    ind = calc_target_index(state, cx, cy) # Calculate index using function calc_target_index

    if pind >= ind: # Update index to next
        ind = pind

    if ind < len(cx): # Set tx and ty as our current trajectory points
        tx = cx[ind]
        ty = cy[ind]
    else:
        tx = cx[-1]
        ty = cy[-1]
        ind = len(cx) - 1

    alpha = math.atan2(ty - state.y, tx - state.x) - state.yaw

    if state.v < 0:  # To always have positive velocity
        alpha = math.pi - alpha

    Lf = k * state.v + Lfc

    delta = math.atan2(2.0 * L * math.sin(alpha) / Lf, 1.0)

    return delta, ind

def calc_target_index(state, cx, cy):

    dx = [state.x - icx for icx in cx] # Search for nearest point index
    dy = [state.y - icy for icy in cy]
    d = [abs(math.sqrt(idx ** 2 + idy ** 2)) for (idx, idy) in zip(dx, dy)]
    ind = d.index(min(d))
    Ln = 0.0

    Lf = k * state.v + Lfc

    # Search for look ahead target point index
    while Lf > Ln and (ind + 1) < len(cx):
        dx = cx[ind + 1] - cx[ind]
        dy = cy[ind + 1] - cy[ind]
        Ln += math.sqrt(dx ** 2 + dy ** 2)
        ind += 1
        print (ind)
    return ind

#######
# ROS #
#######

rospy.init_node('pure_pursuit_controller')
ctrl_pub= rospy.Publisher('/lli/ctrl_request',lli_ctrl_request,queue_size=1)
target_pub = rospy.Publisher('pure_pursuit_target_pose', PointStamped, queue_size=1)


########
# MAIN #
########

traj_x = []
traj_y = []

def callback_mocap(odometry_msg):

    if not len(traj_x) == 0: # Hold callback for mocap until callback for trajectory finishes
        x_pos = odometry_msg.pose.pose.position.x # Mocap data about Car
        y_pos = odometry_msg.pose.pose.position.y
        yaw = odometry_msg.pose.pose.orientation.z
        v = odometry_msg.twist.twist.linear.x

        state_m = State(x_pos, y_pos, yaw, v)

        ind = calc_target_index(state_m, traj_x, traj_y) # Get index from current Mocap data about car


        if ind < len(traj_x)-1:
            print("### RUNNING TRAJECTORY")

            delta, ind =  pure_pursuit_control(state_m, traj_x, traj_y, ind)

            target_pose = PointStamped() # Pure pursuit target pose Re-publishing
            target_pose.header.stamp = rospy.Time.now()
            target_pose.header.frame_id = 'qualisys'
            target_pose.point.x = traj_x[ind]
            target_pose.point.y = traj_y[ind]
            target_pub.publish(target_pose)

            target_angle = max(-80, min(delta / (math.pi / 4) * 100, 80))
            control_request = lli_ctrl_request() # Low level interface settings of speed and angle while running
            control_request.velocity = target_speed
            control_request.steering = target_angle

        else:
            print("### DONE WITH TRAJECTORY")

            control_request = lli_ctrl_request() # Low level interface settings while done
            control_request.velocity = 0
            control_request.steering = 0

        ctrl_pub.publish(control_request)

def callback_traj(traj_msg):

	global traj_x # Store trajectory message in global lists to store data for mocap callback
	global traj_y

	traj = traj_msg.poses
	traj_x, traj_y = [], []

	for traj_pt in traj:
		traj_x.append(traj_pt.position.x)
		traj_y.append(traj_pt.position.y)


def main():
    mocap_sub = rospy.Subscriber('odometry_body_frame', Odometry, callback_mocap)
    traj_sub = rospy.Subscriber('/nav_traj' + '/SVEA5', PoseArray, callback_traj)
    rospy.spin()

if __name__ == '__main__':
    main()
