
#import rospy
#import tf
import math


def curvature(x_car, y_car, x_g, y_g, heading, D):
    x_gv = math.cos(heading)*(x_g - x_car) + math.sin(heading)*(y_g-y_car)
    y_gv = -math.sin(heading)*(x_g - x_car) + math.cos(heading)*(y_g - y_car) #goal to vehicle coordinates
    curv = 2*x_gv/D
    return curv, x_gv, y_gv


def closest_point_goal(x_car, y_car, path, points, last_index): #finds the first point or the next point in the path
    num_path = len(path) #path is a list which has [x,y,heading]
    D = 1 #look ahead distance
    if points == 0: #first point
        d_list = []
        for i in range(num_path):
            dx = path[i][0] - x_car # path_node x - car value x
            dy = path[i][1] - y_car
            dist = math.sqrt(math.pow(dx, 2) + math.pow(dy, 2)) #calculate distance for each point on path
            d_list.append(dist)
        min_ind = d_list.index(min(d_list)) #find min index
        x_goal = path[min_ind][0]
        y_goal = path[min_ind][1]
    else:
        d_list = []
        for i in range(last_index, num_path): #dont iterate over past points travelled
            dx = path[i][0] - x_car
            dy = path[i][1] - y_car
            dist = math.sqrt(math.pow(dx, 2) + math.pow(dy, 2)) -D #Calculate distance to look ahead
            if dist < 0:
                dist = -dist
            d_list.append(dist)
            min_ind = d_list.index(min(d_list))  # find min index
            x_goal = path[min_ind][0]
            y_goal = path[min_ind][1]

    return x_goal, y_goal, min_ind, D


def multi_hop(x_car, y_car, path):
    Goal_reached=False
    last_index=0
    points=0
    dx=path[len(path)][0] - x_car #end point on path distance
    dy= path[len(path)][0] - y_car
    dist = math.sqrt(math.pow(dx, 2) + math.pow(dy, 2))
    x_points=[]
    y_points=[]
    while dist>0.1: #if distance to end point is less than 0.1 then terminate
        x_goal, y_goal, min_ind, D = closest_point_goal(x_car, y_car, path, points, last_index) # closest to hop distance
        curv, x_gv, y_gv = curvature(x_car, y_car, x_goal, y_goal, path[min_ind][2], D) # get the curvature and x_gv, y_gv
        x_car, y_car = got_to_goal(x_car, y_car, curv, x_gv, y_gv) #going to current goal (go_to_goal will be rewritten)
        last_index = min_ind #min_index is the index for previous point
        points=points+1
        x_points.append(x_car)
        y_points.append(y_car)
    return x_points, y_points



############testing with dummy path below########


def got_to_goal(x_car, y_car, curv, x_gv, y_gv): #going to goal for testing with dummy path
    v=1 #constant speed
    dt=0.00001
    vel_x=math.cos(curv)*v
    vel_y = math.sin(curv)*v
    print(vel_y)
    dx = x_gv - x_car
    dy = y_gv - y_car
    dist = math.sqrt(math.pow(dx, 2) + math.pow(dy, 2))
    while dist>0.1:
        x_car = x_car + vel_x * dt
        print(dist)
        y_car = y_car + vel_y * dt
        dx = x_gv - x_car
        dy = y_gv - y_car
        dist = math.sqrt(math.pow(dx, 2) + math.pow(dy, 2))
    return x_car, y_car


#parameters for the testing
x_car=1
y_car=1
heading=10
x_g=2
y_g=3
D=1
curv, x_gv, y_gv = curvature(x_car, y_car, x_g, y_g, heading, D)
print(y_gv)
x_car, y_car = got_to_goal(x_car, y_car, curv, x_g, y_g)
print(x_car)
print(y_car)
