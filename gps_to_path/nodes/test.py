
import rospy
import overpy
import shapely.geometry as geometry
from shapely.prepared import prep
from shapely.ops import linemerge
import os
import utm
import numpy as np
from random import random
import time

import gpxpy
import gpxpy.gpx

import ros_numpy
import tf2_ros
from nav_msgs.msg import Path
from sensor_msgs.msg import NavSatFix, PointCloud2, PointField
from std_msgs.msg import String
from geometry_msgs.msg import Point, Quaternion, PoseStamped, Pose, TransformStamped
import std_msgs.msg
from math import sqrt, sin, cos, pi
from tf.transformations import quaternion_from_euler, quaternion_from_matrix, quaternion_multiply, euler_from_quaternion, quaternion_inverse, inverse_matrix, quaternion_matrix
from numpy import deg2rad, rad2deg, genfromtxt
import os.path
from tf.transformations import euler_from_quaternion

polygon = geometry.Polygon([[1,1],[2,2],[3,2],[3,0]])

xmin, ymin, xmax, ymax = polygon.bounds
density = 10
x = np.arange(np.floor(xmin * density) / density, np.ceil(xmax * density) / density + 1/density, 1 / density) 
y = np.arange(np.floor(ymin * density) / density, np.ceil(ymax * density) / density + 1/density, 1 / density)
xv,yv = np.meshgrid(x,y)
xv = xv.ravel()
yv = yv.ravel()
points = geometry.MultiPoint(np.array([xv,yv]).T).geoms

polygon = prep(polygon)

contains = lambda p: polygon.contains(p)

ret = filter(contains, points)
ret1 = list(ret)

self.pcd_points = list(points)
self.pcd_points = np.array(list(geometry.LineString(self.pcd_points).xy)).T




