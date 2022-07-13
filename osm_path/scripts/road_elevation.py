import sjtsk_to_utm
import numpy as np
import geopy.distance
from os import path
from math import ceil, dist
import shapely.geometry as geom
from matplotlib import pyplot as plt
import time


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
    #print(waypoints)

    return waypoints


def road_gps_to_csv(road, waypoints_density=0.1, waypoints_file="waypoints.csv", circular=False):
    #print(road)
    coords = np.array([[node[1], node[2]] for node in road])
    #print(coords)
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


def find_steep_roads(elev_data: list):
    # TODO: multiple levels of road steepnes -> change in height in corelation with change in length
    network_steepness = []
    for road in elev_data:
        road_steepness = []
        for i in range(len(road)-1):
            road_steepness.append((dist(road[i][:2], road[i+1][:2]), road[i+1][2]-road[i][2]))
        network_steepness.append(road_steepness)

    return network_steepness


def road_cost_for_height(elev_data: list):
    # TODO: final method -> returns cost of traversing here regarding height visibility
    return


def visualize_3D(data):
    #DEBUG function
    fig = plt.figure()
    ax = fig.gca(projection='3d')
    for road in data:
        ax.scatter(road[:,0], road[:,1], road[:,2])
    ax.set_xlabel('Easting [m]')
    ax.set_ylabel('Northing [m]')
    ax.set_zlabel('Altitude [m]')
    plt.show()
    