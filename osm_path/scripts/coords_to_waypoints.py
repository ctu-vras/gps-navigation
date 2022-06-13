import geopy.distance
import numpy as np
from math import floor, ceil

def coords_to_waypoints(waypoints_density = 0.1, coords_file="coords.csv", waypoints_file="waypoints.csv"):     # waypoints_density == dist between neighbor waypoints. in meters
    coords = np.genfromtxt(coords_file, delimiter=',')
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

    np.savetxt(waypoints_file, waypoints, delimiter=',')

    print("Generated {} waypoints from {} coordinate pairs.".format(len(waypoints), len(coords)))
    
    return original_waypoint_indices


if __name__=='__main__':
    coords_to_waypoints()

