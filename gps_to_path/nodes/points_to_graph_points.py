from shapely.geometry import Point, LineString, MultiPoint
import numpy as np
from math import cos,sin,atan2,pi,ceil
import time
import matplotlib.pyplot as plt
import rospy

def get_point_line(p1,p2,density,increase=0):
    x1 = p1.x
    y1 = p1.y
    x2 = p2.x
    y2 = p2.y

    a = np.array([x1,y1])
    b = np.array([x2,y2])

    vec = (b-a).T

    line = get_equidistant_points(a,b,int(ceil(np.linalg.norm(vec)/density))+1)  # ceil, because more points is OK, while less points could be problematic (distance between points should not be larger than 1*density)
    dist_line = np.zeros((len(line),1))

    if increase>0:
        line,dist_line = increase_line(line,dist_line,b-a,increase,density)

    return vec,line,dist_line

def increase_line(line,dist_line,vec,n,density):
    vec = vec/np.linalg.norm(vec)
    arange_increase_vec = density * np.arange(1,n+1)
    arange_increase_vec = arange_increase_vec.reshape((-1,1))
    increase_vec = arange_increase_vec * vec
    before = line[0] * np.ones((n,2)) - increase_vec
    before = np.flip(before,axis=0)
    after = line[-1] * np.ones((n,2)) + increase_vec

    line = np.concatenate((before,line,after),axis=0)
    dist_line = np.concatenate((np.flip(arange_increase_vec),dist_line,arange_increase_vec),axis=0)
    return line,dist_line

def get_equidistant_points(p1, p2, n):
    return np.concatenate((np.expand_dims(np.linspace(p1[0], p2[0], max(n,2)), axis=1), np.expand_dims(np.linspace(p1[1], p2[1], max(n,2)), axis=1)),axis=1)

def points_arr_to_point_line(points,density):
    """ Input: array of (shapely) Points, density
        Output: array of (shapely) Points where the distance between neighboring points is (around) density. """
    point_line = np.zeros((1,2))
    original_waypoint_indices = []

    for i in range(len(points)-1):
        p1 = points[i]
        p2 = points[i+1]

        original_waypoint_indices.append(point_line.shape[0]-1)

        _,line,_ = get_point_line(p1,p2,density)
        
        if not i >= len(points)-2:
            line = line[:-1]

        point_line = np.append(point_line,line,axis=0)

    point_line = point_line[1:,:]
    original_waypoint_indices.append(point_line.shape[0]-1)

    return list(map(Point, zip(point_line[:,0], point_line[:,1]))),original_waypoint_indices


def points_to_graph_points(point1, point2, density=1, width=10):

    perpendicular_increase = int(round(width/2/density))
    parallel_increase = int(round(width/4/density))
    
    if point1.bounds == point2.bounds:
        return MultiPoint([point1.x, point1.y]),MultiPoint([point1.x, point1.y]),np.zeros((1,1))
    else:
        vec,point_line,dist_line = get_point_line(point1,point2,density,parallel_increase)

    normal_vec = np.matmul(np.array([[cos(pi/2),-sin(pi/2)],[sin(pi/2),cos(pi/2)]]), vec)/np.linalg.norm(vec)

    points_in_line = point_line.shape[0]

    all_points = np.zeros((points_in_line*(perpendicular_increase*2+1),2))
    dist_from_line = np.zeros((points_in_line*(perpendicular_increase*2+1),1))
    #start_index = points_in_line*num_points + increase      # index of the original point 1 (start)
    #goal_index = points_in_line*(num_points+1)-1 - increase # index of the original point 2 (goal)

    line_start_index = points_in_line*perpendicular_increase       
    line_end_index = points_in_line*(perpendicular_increase+1)-1  

    all_points[line_start_index:line_end_index+1,:] = point_line
    dist_from_line[line_start_index:line_end_index+1] = dist_line

    for i in range(perpendicular_increase):
        pos_line_points = point_line + normal_vec*density*(i+1)
        neg_line_points = point_line - normal_vec*density*(i+1)
        all_points[points_in_line*(perpendicular_increase+i+1):points_in_line*(perpendicular_increase+i+2),:] = pos_line_points
        all_points[points_in_line*(perpendicular_increase-i-1):points_in_line*(perpendicular_increase-i),:] = neg_line_points
        dist_from_line[points_in_line*(perpendicular_increase+i+1):points_in_line*(perpendicular_increase+i+2)] = dist_line + (i + 1) * density
        dist_from_line[points_in_line*(perpendicular_increase-i-1):points_in_line*(perpendicular_increase-i)] = dist_line + (i + 1) * density
    
    """ if increase:
        plt.scatter(all_points[:,0], all_points[:,1])
        plt.savef
        ig("{}.pdf".format(time.time())) """

    #all_points = list(map(Point, zip(all_points[:,0], all_points[:,1])))
    #rospy.logwarn(type(all_points))
    #rospy.logwarn(all_points.shape)
    #rospy.logwarn(all_points[:5])

    #np.save("weird_arr",all_points)
    #all_points = np.load("weird_arr.npy")

    all_points = MultiPoint(all_points)
    point_line = MultiPoint(point_line)

    return all_points,point_line,dist_from_line
