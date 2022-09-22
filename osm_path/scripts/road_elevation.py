import sjtsk_to_utm
import numpy as np
from math import ceil, dist, floor
import shapely.geometry as geom
from threading import Thread

from road_crossing_consts import *


def generate_waypoints(road: geom.LineString, waypoints_density: float, circular: bool = False) -> list:
    '''
    Generates points for road so that the maximal distnace between two adjacent
    points is waypoint_density.

    Parameters
    ----------
    road: shapely.geometry.LineString
        LineString representation of road.
    waypoint_density: float
        Maximal distance between two adjacent points.
    circular: bool
        Is the represented road closed, e.g. a roundabout, ... Base value -> False.
    '''
    coords = np.array(road.coords)
    if circular:  # closed way
        coords = np.append(coords, [coords[0,:]],axis=0)

    waypoints = []

    for i in range(len(coords)-1):
        c1 = coords[i]
        c2 = coords[i+1]

        dist_meters = dist(c1, c2)
        distance = c2-c1
        steps = ceil(dist_meters/waypoints_density)
        increment = distance/steps

        for j in range(steps):
            waypoints.append(tuple(c1+j*increment))
        
    waypoints.append(tuple(c2))  # the final point has to be added manually.

    return waypoints


def get_road_network_elevation(road_network: geom.MultiLineString, elev_data_files: list) -> list:
    '''
    Returns list with assigned heights for roads. Maximal distance between two adjacent data points is 1 m.

    Parameters
    ----------
    road_network: shapely.geometry.MultiLineString
        All roads in target area represented as MultiLineString.
    elev_data_files: list
        List with paths to all neccessary files containing elevation data.
    '''
    # TODO: multiple input files of elevation data
    data_file = "/home/vlkjan6/Documents/gps-navigation/gps-navigation/osm_path/scripts/PRAH72_5g.xyz"

    network_elev = []
    road_nodes_count = []
    waypoints = []
    road_nodes = []
    road_way = []

    for road in list(road_network.geoms):
        waypoints.extend(generate_waypoints(road, 1))
        road_nodes_count.append(len(waypoints)-sum(road_nodes_count))
        if sum(road_nodes_count) > NODES_IN_BATCH:
            road_nodes.append(road_nodes_count)
            road_way.append(waypoints)
            road_nodes_count = []
            waypoints = []
    if road_nodes_count:  # if not empty add last batch
        road_nodes.append(road_nodes_count)
        road_way.append(waypoints)

    data_arr = []
    for waypoints in road_way:
        data_arr.append(sjtsk_to_utm.DataPoints(waypoints, data_file, n_closest=5))

    elev_data = [[] for i in range(len(road_nodes))]
    def run(data: sjtsk_to_utm.DataPoints, elev_data: list, index: int):
        elev_data[index] = data.run()

    threads = []
    for i in range(len(data_arr)):
        t = Thread(target=run, args=(data_arr[i], elev_data, i))
        threads.append(t)
        t.start()
    for t in threads:
        t.join()

    for i in range(len(road_nodes)):
        prev_nodes = 0
        for num_nodes in road_nodes[i]:
            network_elev.append(elev_data[i][prev_nodes:prev_nodes+num_nodes])
            prev_nodes += num_nodes
    print("INFO: processed {} height points.".format(sum(sum(road_nodes) for road_nodes in road_nodes)))

    return network_elev


def classify_TPI(elev_data: list) -> list:
    '''
    Classify individual segments according to TPI classes, with changed threshold values.

    Parameters
    ----------
    elev_data: list
        Road network represented as points (x,y,z).
    '''
    n_small = SMALL_NEIGH
    n_large = LARGE_NEIGH
    s_change = SMALL_CHANGE
    l_change = LARGE_CHANGE
    network_classification = []

    for road in elev_data:
        road_classification = []
        for seg_num in range(len(road)-1):
            small = sum(road[(seg_num-n_small if seg_num-n_small > 0 else 0): \
                             (seg_num+1+n_small if seg_num+1+n_small < len(road) else len(road)), 2]) - road[seg_num, 2]
            small = road[seg_num][2] - small/((n_small if seg_num+1+n_small <= len(road) else len(road)-seg_num-1) + \
                                              (n_small if seg_num-n_small >= 0 else seg_num))
            large = sum(road[(seg_num-n_large if seg_num-n_large > 0 else 0): \
                             (seg_num+1+n_large if seg_num+1+n_large < len(road) else len(road)), 2]) - road[seg_num, 2]
            large = road[seg_num][2] - large/((n_large if seg_num+1+n_large <= len(road) else len(road)-seg_num-1) + \
                                              (n_large if seg_num-n_large >= 0 else seg_num))

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

            road_classification.append((road[seg_num:seg_num+2, :2], classify))
        network_classification.append(road_classification)
    return network_classification


def road_cost_for_height(network_classification: list, exploration_limit: float = 100) -> list:
    '''
    Returns segments with their assigned costs.

    Parameters
    ----------
    network_calssification: list
        Road segments with their TPI class.
    exploration_limit: float
        Determines what distance in meters in both direction of road we will explore. Base value -> 100.
    '''
    class_costs = [CANYONS, MIDSLOPE_DRAIN, UPLAND_DRAIN, U_VALLEY, PLAINS, OPEN_SLOPES, \
                   UPPER_SLOPES, LOCAL_RIDGE, MIDSLOPE_RIDGE, MOUNTAIN_TOP]
    ranked_segments = []
    for road in network_classification:
        big_seg = []
        for i in range(len(road)):
            seg_value = 0
            dist1 = dist(road[i][0][0], road[i][0][1])/2 
            for j in range(i, -1, -1):  # from segment to start
                seg_value += class_costs[road[j][1]-1]*dist(road[j][0][0], road[j][0][1]) 
                dist1 += dist(road[j][0][0], road[j][0][1]) 
                if dist1 >= exploration_limit:
                    break
            dist2 = dist(road[i][0][0], road[i][0][1])/2 
            for j in range(i, len(road)):  # from segment to end
                seg_value += class_costs[road[j][1]-1]*dist(road[j][0][0], road[j][0][1]) 
                dist2 += dist(road[j][0][0], road[j][0][1]) 
                if dist2 >= exploration_limit:
                    break
            seg_value /= (dist1+dist2)
            seg_value = floor(seg_value) if seg_value < ROAD_ELEVATION_RANKS else ROAD_ELEVATION_RANKS
            # Combine adjacent segments with the same value into one bigger segment
            if not big_seg:
                big_seg = ([road[i][0].tolist()], seg_value)
            elif seg_value == big_seg[1]:
                # Is the next segment adjacent
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
 