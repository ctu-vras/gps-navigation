import gpxpy
import gpxpy.gpx
import geopy.distance
import numpy as np
from math import floor, ceil


# waypoints_density == dist between neighbor waypoints. in meters
def gpx_to_waypoints(waypoints_density=0.1, gpx_file="coords.gpx", waypoints_file="waypoints.csv", circular=False):
    if waypoints_density <= 0.:
        waypoints_density = 0.01
    gpx_f = open(gpx_file, 'r')
    gpx_object = gpxpy.parse(gpx_f)
    coords = np.array([[point.latitude, point.longitude]
                       for point in gpx_object.waypoints])
    if circular:
        coords = np.append(coords, [coords[0, :]], axis=0)

    waypoints = []

    for i in range(len(coords)-1):
        c1 = coords[i]
        c2 = coords[i+1]

        dist_meters = geopy.distance.geodesic(c1, c2).m
        dist_degs = c2-c1
        increment = dist_degs*waypoints_density/dist_meters

        for j in range(int(ceil(dist_meters/waypoints_density))):
            waypoints.append(c1+j*increment)

    waypoints.append(c2)    # The final point has to be added manually.

    if waypoints_file[-3:] == "gpx":
        save_path_as_gpx(waypoints_file, waypoints)
    else:
        np.savetxt(waypoints_file, waypoints, delimiter=',')

    print("Generated {} waypoints from {} coordinate pairs.".format(
        len(waypoints), len(coords)))

def save_path_as_gpx(fn,waypoints):
    gpx = gpxpy.gpx.GPX()

    for point in waypoints:
        gpx.waypoints.append(gpxpy.gpx.GPXWaypoint(point[0], point[1]))

    xml = gpx.to_xml()
    with open(fn, "w") as f:
        f.write(xml)
        f.close()

if __name__ == "__main__":
    gpx_to_waypoints(waypoints_density=0.1, gpx_file="/home/robot/0804_gps/src/gps-navigation/osm_path_planner/data/kn.gpx",
                     waypoints_file="/home/robot/0804_gps/src/gps-navigation/osm_path_planner/data/waypoints.gpx", circular=True)
