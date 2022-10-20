import matplotlib.pyplot as plt
import numpy as np
import pyproj
from geodesy import utm
import copy


class DataPoints():
    def __init__(self, waypoints, fn_data_points, n_closest):
        """ Prepare waypoints to analyze. """
        self.waypoints = waypoints
        self.waypoints = np.pad(self.waypoints, ((0,0), (0,1)), constant_values=(0,0))  # add a column of zeros

        self.max_easting = np.max(self.waypoints[:,0])
        self.min_easting = np.min(self.waypoints[:,0])
        self.max_northing = np.max(self.waypoints[:,1])
        self.min_northing = np.min(self.waypoints[:,1])

        """ Prepare data. """
        self.data_points = np.genfromtxt(fn_data_points, delimiter=',')
        
        """ Parameter for height assignment. """
        self.n_closest = n_closest

    def waypoints_to_utm(self):
        for i,waypoint in enumerate(self.waypoints):
            utm_coords = utm.fromLatLong(waypoint[0],waypoint[1])
            waypoint = np.array([utm_coords.easting, utm_coords.northing])
            self.waypoints[i] = waypoint

    def narrow_region(self):
        mask =  (self.data_points[:,0] > self.min_easting-5)  & \
                (self.data_points[:,0] < self.max_easting+5)  & \
                (self.data_points[:,1] > self.min_northing-5) & \
                (self.data_points[:,1] < self.max_northing+5)

        self.data_points = self.data_points[mask]
    
    def closest_points(self, point, points, n):
        """ Find the n closest points to point and their distances. """
        points = np.asarray(points)
        dist2 = np.sum((points - point)**2, axis=1)
        n = n if len(dist2) > n else len(dist2)-1
        if n == 0:
            arg_n_min_dist = [0]
        else:
            arg_n_min_dist = np.argpartition(dist2,n)[:n]
        #n_closest_points = np.array(points)[arg_n_min_dist]
        distances = np.sqrt(dist2[arg_n_min_dist])
        return [arg_n_min_dist,distances]

    def assign_height(self):
        """ To each waypoint assign height as the weighted mean of heights of the n
        closest points, where the weights are 1/distance of point from waypoint."""
        for i,point in enumerate(self.waypoints):
            arg_n_min_dist,distances = self.closest_points(point[0:2], self.data_points[:,0:2], self.n_closest)
            n_closest_points = self.data_points[arg_n_min_dist]
            heights = n_closest_points[:,2]
            weights = np.reciprocal(distances)
            height = np.sum(np.multiply(heights,weights/np.sum(weights)))
            self.waypoints[i,2] = height

    def visualize(self):
        fig = plt.figure()
        ax = fig.gca(projection='3d')
        ax.scatter(self.waypoints[:,0], self.waypoints[:,1], self.waypoints[:,2])
        ax.set_xlabel('Easting [m]')
        ax.set_ylabel('Northing [m]')
        ax.set_zlabel('Altitude [m]')
        plt.show()
    
    def plot_points(self):
        plt.scatter(self.data_points[:,0], self.data_points[:,1], c='blue', alpha = 0.5)
        plt.scatter(self.waypoints[:,0], self.waypoints[:,1], c='red', alpha = 0.5)
        plt.show()

    def run(self):
        self.narrow_region()
        self.assign_height()
        #self.plot_points()
        #self.visualize()
        return copy.deepcopy(self.waypoints)
