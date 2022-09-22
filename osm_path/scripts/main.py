import osm_analysis 
import os
import sys
from background_map import get_background_image

from geodesy import utm
import gpxpy.gpx

import matplotlib.pyplot as plt
import matplotlib.patches as ptch
import numpy as np
import pandas as pd

road_cross_cost = False
if len(sys.argv) > 1:
    if "-c" in sys.argv:
        COORDS_FILE = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "gpx/"+str(sys.argv[sys.argv.index("-c")+1]))
    else:
        COORDS_FILE = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "gpx/cimicky.gpx")
    if "-o" in sys.argv:
        PATH_NAME = str(sys.argv[sys.argv.index("-o")+1])
    else:
        PATH_NAME = "path"
    if "-r" in sys.argv:
        road_cross_cost = True
        print("good")
else:
    COORDS_FILE = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "gpx/cimicky.gpx")
    PATH_NAME = "path"

if not PATH_NAME.isalnum():
    PATH_NAME = "path"
    print("Path name defaulted to \'path\'. (Use only letters and numbers, please).")

# ///////////////////////
# / Plotting functions. /
# ///////////////////////

def plot_path(path, ax):
    step_size = 1
    ax.scatter(path[::step_size,0], path[::step_size,1], color='#000000', alpha=0.8, s=3, marker='o', zorder = 18000)
    ax.scatter(path[::step_size,0], path[::step_size,1], color='#50C2F6', alpha=0.8, s=2, marker='o', zorder = 20000)

def plot_barrier_areas(barrier_areas, ax):
    for area in barrier_areas:
        x,y = area.line.exterior.xy
        ax.plot(x, y, c='#BF0009', linewidth=1, zorder = 7)
        
        if area.in_out != "inner":
            ax.fill(x,y,c='#BF0009', alpha=0.4, zorder = 5)

def plot_road_costs(road_polygons, ax):
    colors = ["blue", "deepskyblue", "aqua", "turquoise", "limegreen", "green", "yellow", "gold",\
              "orange", "peru", "orangered", "firebrick", "deeppink", "magenta", "purple"]
    for road_level in range(len(road_polygons)):
        for road in road_polygons[road_level]:
            x,y = road.exterior.xy
            ax.fill(x, y, c=colors[road_level], alpha=0.6, zorder=6)

def plot_legend(ax): 
    blue_patch = ptch.Patch(color="blue", label="Level 00")
    deepskyblue_patch = ptch.Patch(color="deepskyblue", label="Level 01")
    aqua_patch = ptch.Patch(color="aqua", label="Level 02")
    turquoise_patch = ptch.Patch(color="turquoise", label="Level 03")
    limegreen_patch = ptch.Patch(color="limegreen", label="Level 04")
    green_patch = ptch.Patch(color="green", label="Level 05")
    yellow_patch = ptch.Patch(color="yellow", label="Level 06")
    gold_patch = ptch.Patch(color="gold", label="Level 07")
    orange_patch = ptch.Patch(color="orange", label="Level 08")
    tomato_patch = ptch.Patch(color="peru", label="Level 09")
    orangered_patch = ptch.Patch(color="orangered", label="Level 10")
    red_patch = ptch.Patch(color="firebrick", label="Level 11")
    deeppink_patch = ptch.Patch(color="deeppink", label="Level 12")
    magenta_patch = ptch.Patch(color="magenta", label="Level 13")
    purple_patch = ptch.Patch(color="purple", label="Level 14")
    obstacle_patch = ptch.Patch(color="#BF0009", label="Obstacle")
    path_patch = ptch.Patch(color="#50C2F6", label="Path")
    handles = [blue_patch, deepskyblue_patch, aqua_patch, turquoise_patch, limegreen_patch, green_patch,\
              yellow_patch, gold_patch, orange_patch, tomato_patch, orangered_patch, red_patch, deeppink_patch,\
              magenta_patch, purple_patch, obstacle_patch, path_patch]
    ax.legend(handles=handles, loc="center left", bbox_to_anchor=(1, 0.5))

def get_margin(min_long,max_long,min_lat,max_lat):
    y_margin = (max_lat-min_lat) * 0.1
    x_margin = (max_long-min_long) * 0.1

    if y_margin < x_margin:
        y_margin = x_margin
    else:
        x_margin = y_margin
    
    return x_margin, y_margin

def plot_background_map(ax, image):

    min_utm = utm.fromLatLong(min_lat - y_margin, min_long - x_margin)
    max_utm = utm.fromLatLong(max_lat + y_margin, max_long + x_margin)
    ax.imshow(image, extent = [min_utm.easting,\
                                max_utm.easting,\
                                min_utm.northing,\
                                max_utm.northing], alpha = 1, zorder = 0)

    ax.set_ylim([min_utm.northing, max_utm.northing])
    ax.set_xlim([min_utm.easting, max_utm.easting])

# ///////////////////////
# //// Running code. ////
# ///////////////////////

# Run the graph search. Save generated path to a .gpx file.
path_analysis = osm_analysis.PathAnalysis(COORDS_FILE)
path_analysis.run_standalone(os.path.join(os.path.dirname(os.path.dirname(__file__)), "path/{}.gpx".format(PATH_NAME)), road_cross_cost)

# Extract variables.
roads =                 np.array(list(path_analysis.roads))
road_polygons =         path_analysis.road_polygons
footways =              np.array(list(path_analysis.footways))
barriers =              np.array(list(path_analysis.barriers))

min_x =                 path_analysis.min_x
min_y =                 path_analysis.min_y
max_x =                 path_analysis.max_x   
max_y =                 path_analysis.max_y

min_lat =               path_analysis.min_lat
min_long =              path_analysis.min_long
max_lat =               path_analysis.max_lat
max_long =              path_analysis.max_long

path =                  path_analysis.path

fig_fn = os.path.join(os.path.dirname(os.path.dirname(__file__)), "path/{}.png".format(PATH_NAME))
print("Plotting and saving to {}".format(fig_fn))
x_margin, y_margin = get_margin(min_long,max_long,min_lat,max_lat)
background_map = get_background_image(min_long, max_long, min_lat, max_lat, x_margin, y_margin)

fig, ax = plt.subplots(figsize=(12,12), dpi=400)
N = 100

plot_background_map(ax, background_map)
plot_barrier_areas(barriers,ax)
plot_road_costs(road_polygons, ax)
plot_path(path,ax)
if road_cross_cost:
    plot_legend(ax)

ax.set_aspect('equal', adjustable='box')
ax.set_xlabel('Easting (m)')
ax.set_ylabel('Northing (m)')
plt.savefig(fig_fn)
print("Done running. OK.")
#plt.show()