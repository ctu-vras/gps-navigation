import sjtsk_to_utm
import numpy as np
import geopy.distance
from os import path
from math import ceil, dist, floor
import shapely.geometry as geom
import time
from road_crossing_consts import *


def generate_waypoints(road, waypoints_density, circular=False):
    coords = list(road.coords)
    if circular:
        coords = np.append(coords, [coords[0,:]],axis=0)

    waypoints = []
    original_waypoint_indices = []

    for i in range(len(coords)-1):
        c1 = coords[i]
        c2 = coords[i+1]

        dist_meters = geom.Point(c1).distance(geom.Point(c2))
        dist = c2[0]-c1[0], c2[1]-c1[1]
        steps = ceil(dist_meters/waypoints_density)
        increment = dist[0]/steps, dist[1]/steps

        original_waypoint_indices.append(len(waypoints))

        for j in range(steps):
            waypoints.append(tuple([c1[0]+j*increment[0],c1[1]+j*increment[1]]))
        
    original_waypoint_indices.append(len(waypoints))
    waypoints.append(c2)    # The final point has to be added manually.

    return waypoints


def road_gps_to_csv(road, waypoints_density=0.1, waypoints_file="waypoints.csv", circular=False):
    coords = np.array([[node[1], node[2]] for node in road])
    if circular:
        coords = np.append(coords,[coords[0,:]],axis=0)

    waypoints = []
    original_waypoint_indices = []
    
    for i in range(len(coords)-1):
        c1 = coords[i]
        c2 = coords[i+1]

        dist_meters = geopy.distance.geodesic(c1,c2).m
        dist_degs = c2-c1
        increment = dist_degs*waypoints_density/dist_meters

        original_waypoint_indices.append(len(waypoints))
        
        for j in range(ceil(dist_meters/waypoints_density)):
            waypoints.append(c1+j*increment)
        
    original_waypoint_indices.append(len(waypoints))
    waypoints.append(c2)    # The final point has to be added manually.

    #np.savetxt(waypoints_file, waypoints, delimiter=',')
    
    print("Generated {} waypoints from {} coordinate pairs.".format(len(waypoints), len(coords)))

    return original_waypoint_indices


def get_road_elevation(road):
    waypoints = generate_waypoints(road, 10)

    """ ZABAGED - data z katastru
        4g ..... 5m dense square lattice
        5g ..... irrelugar, newer, better """
    fn_4g = "553988_CVUT/4g/PRAH72_4g.xyz"
    fn_5g = "553988_CVUT/5g/PRAH72_5g.xyz"

    fn = fn_5g

    if not path.exists(fn[:-3]+"csv"):
        transformer = sjtsk_to_utm.sjtsk2utm(fn)
        transformer.run()
        transformer.save_to_file()
        transformer.plot()

    data = sjtsk_to_utm.DataPoints(waypoints, fn[:-3]+"csv", n_closest=5)
    waypoints = data.run()
    return waypoints


def get_road_network_elevation(road_network: geom.MultiLineString, elev_data_files: list):
    '''start_t = time.time()
    data_file = "elev_data.csv"
    for file_name in elev_data_files:
        transformer = sjtsk_to_utm.sjtsk2utm(file_name, data_file)
        transformer.run()
        transformer.save_to_file()
    print("DEBUG: Elev data file: %.05f" % (time.time()-start_t))'''
    data_file = "/home/vlkjan6/Documents/RobinGas/testfolder/553988_CVUT/5g/PRAH72_5g.xyz"

    network_elev = []
    road_nodes = []
    waypoints = []

    start_t = time.time()
    for road in list(road_network.geoms):
        waypoints.extend(generate_waypoints(road, 1))
        road_nodes.append(len(waypoints)-sum(road_nodes))
    data = sjtsk_to_utm.DataPoints(waypoints, data_file, n_closest=5)
    print("DEBUG: Create DataPoints: %.05f" % (time.time()-start_t))
    start_t = time.time()
    elev_data = data.run()
    print("DEBUG: Get elev: %.05f" % (time.time()-start_t))
    prev_nodes = 0
    for num_nodes in road_nodes:
        network_elev.append(elev_data[prev_nodes:prev_nodes+num_nodes])
        prev_nodes += num_nodes
    print("INFO: processed {} height points.".format(sum(road_nodes)))
    return network_elev


def classify_TPI(elev_data: list):
    # TODO: Find best variables for our usecase. n_small and n_large probably set
    # What about s_change and l_change? Should they be the same, different, how, ...
    n_small = SMALL_NEIGH
    n_large = LARGE_NEIGH
    s_change = SMALL_CHANGE
    l_change = LARGE_CHANGE
    network_classification = []
    num_class = [0 for i in range(10)]
    for road in elev_data:
        road_classification = []
        for seg_num in range(len(road)-1):
            small = sum(road[seg_num+1:(seg_num+1+n_small if seg_num+1+n_small < len(road) else len(road)), 2])
            small = road[seg_num][2] - small/(n_small if seg_num+1+n_small < len(road) else len(road)-seg_num-1)
            large = sum(road[seg_num+1:(seg_num+1+n_large if seg_num+1+n_large < len(road) else len(road)), 2])
            large = road[seg_num][2] - large/(n_large if seg_num+1+n_large < len(road) else len(road)-seg_num-1)

            if small <= -s_change:
                classify = 0
            elif small > -s_change and small < s_change:
                classify = 3
            else:
                classify = 7

            if large <= -l_change:
                classify += 1
            elif large > -l_change and large < l_change:
                if classify == 3:
                    slope = road[seg_num+1][2]-road[seg_num][2]/dist(road[seg_num+1][:2], road[seg_num][:2])
                    classify = 5 if slope <= 5 else 6
                else:
                    classify += 2
            else:
                classify += 3 if classify != 3 else 4

            num_class[classify-1] += 1
            road_classification.append((road[seg_num:seg_num+2, :2], classify))
        network_classification.append(road_classification)
    print("DEBUG: TPI classification counts: {}.".format(num_class))
    return network_classification


def road_cost_for_height(network_classification: list, exploration_limit: int = 100):
    class_costs = [CANYONS, MIDSLOPE_DRAIN, UPLAND_DRAIN, U_VALLEY, PLAINS, OPEN_SLOPES, \
                   UPPER_SLOPES, LOCAL_RIDGE, MIDSLOPE_RIDGE, MOUNTAIN_TOP]
    ranked_segments = []
    for road in network_classification:
        big_seg = []
        for i in range(len(road)):
            seg_value = 0
            dist1 = dist(road[i][0][0], road[i][0][1])/2 
            for j in range(i, -1, -1):
                seg_value += class_costs[road[j][1]-1]*dist(road[j][0][0], road[j][0][1]) 
                dist1 += dist(road[j][0][0], road[j][0][1]) 
                if dist1 >= exploration_limit:
                    break
            dist2 = dist(road[i][0][0], road[i][0][1])/2 
            for j in range(i, len(road)):
                seg_value += class_costs[road[j][1]-1]*dist(road[j][0][0], road[j][0][1]) 
                dist2 += dist(road[j][0][0], road[j][0][1]) 
                if dist2 >= exploration_limit:
                    break
            seg_value /= (dist1+dist2)
            seg_value = floor(seg_value) if seg_value < ROAD_ELEVATION_RANKS else ROAD_ELEVATION_RANKS
            if big_seg == []:
                big_seg = ([road[i][0].tolist()], seg_value)
            elif seg_value == big_seg[1]:
                if road[i][0].tolist()[0] in [elem for arr in big_seg[0] for elem in arr] or \
                   road[i][0].tolist()[1] in [elem for arr in big_seg[0] for elem in arr]:
                    big_seg[0].append(road[i][0].tolist())
                else:
                    ranked_segments.append((geom.LineString([elem for arr in big_seg[0] for elem in arr]), big_seg[1]))
                    big_seg = ([road[i][0].tolist()], seg_value)
            else:
                ranked_segments.append((geom.LineString([elem for arr in big_seg[0] for elem in arr]), big_seg[1]))
                big_seg = ([road[i][0].tolist()], seg_value)
        ranked_segments.append((geom.LineString([elem for arr in big_seg[0] for elem in arr]), big_seg[1]))
    return ranked_segments
 