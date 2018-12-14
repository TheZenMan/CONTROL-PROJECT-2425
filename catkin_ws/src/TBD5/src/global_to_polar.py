#! /usr/bin/env python

import rospy # Import the Python library for ROS
import tf
import numpy as np
from math import degrees, cos, sin, sqrt, pi, atan2, sqrt
from nav_msgs.msg import Odometry # Import the Odometry message from the nav_msgs.msg package
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Pose, PoseArray, PoseWithCovarianceStamped, PolygonStamped, Point32

obs_pub = rospy.Publisher('global_to_polar',PoseArray, queue_size=1) 

WAYPOINT_SEPARATION = 0.1
total_points_x = []
total_points_y = []
dist_list = []
phi_list = []

def gen_line_segment(start_pt, end_pt):
        """
        Generates static line segment from start_pt to end_pt
        start_pt (geometry_msgs/Point), end_pt (geometry_msgs/Point)
        """

        cx, cy = [], []

        # compute additional waypoints
        dst = np.linalg.norm(np.asarray([end_pt.x, end_pt.y]) - \
                             np.asarray([start_pt.x, start_pt.y]))
        angle = np.arctan2(end_pt.y - start_pt.y, end_pt.x - start_pt.x)

        waypts = np.arange(0, dst, WAYPOINT_SEPARATION)
        if len(waypts) > 0:
            waypts_x = waypts * cos(angle) + start_pt.x * np.ones(waypts.shape)
            waypts_y = waypts * sin(angle) + start_pt.y * np.ones(waypts.shape)

            # add waypoints
            cx =  np.append(cx, waypts_x)
            cy = np.append(cy, waypts_y)
        else:
            print("######## WARNING: GENERATED EMPTY TRAJECTORY ########")

        return cx, cy

def callback_odom(odom_msg): # Unpacking odom_msg
    global x_pos, y_pos, yaw

    x_pos = odom_msg.pose.pose.position.x
    y_pos = odom_msg.pose.pose.position.y
    yaw = odom_msg.pose.pose.orientation.z

def callback_obs(obs_msg):
    global p1, p2, p3, p4
    global phi_list
    global dist_list

    phi_list = []
    dist_list = []

    p1 = obs_msg.polygon.points[0] # Unpacking the points
    p2 = obs_msg.polygon.points[1]
    p3 = obs_msg.polygon.points[2]
    p4 = obs_msg.polygon.points[3]

    cx1, cy1 = gen_line_segment(p1, p2) # Connecting points with line and generate points on the line
    cx2, cy2 = gen_line_segment(p2, p3)
    cx3, cy3 = gen_line_segment(p3, p4)
    cx4, cy4 = gen_line_segment(p4, p1)

    total_points_x = [cx1,cx2,cx3,cx4]
    total_points_y = [cy1,cy2,cy3,cy4]
    for i in range(len(total_points_x)): # Converting from global to local
        x_rel = cos(yaw)*total_points_x[i] + sin(yaw)*total_points_y[i]
        y_rel = cos(yaw)*total_points_y[i] - sin(yaw)*total_points_x[i]
        phi = atan2(x_rel/y_rel)
        dist = sqrt(pow(x_rel, 2) + pow(y_rel, 2))
        phi_list.append(phi)
        dist_list.append(dist)


    dynamic_obs_polygon = PoseArray()
    dynamic_obs_polygon.header.stamp = rospy.Time.now()
    dynamic_obs_polygon.header.frame_id ='qualisys'

    for i in range(len(phi_list)): # Publishing as PoseArray
        pose = Pose()
        pose.position.x = phi_list[i]
        pose.position.y = dist_list[i]
        dynamic_obs_polygon.poses.append(pose)
    obs_pub.publish(dynamic_obs_polygon)




def main():

    obstacle_sub = rospy.Subscriber('obs_model_dyn', PolygonStamped, callback_obs)
    odom_sub = rospy.Subscriber('odometry_body_frame', Odometry, callback_odom)
    rospy.spin()

if __name__ == '__main__':
    main()



