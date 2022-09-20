import shapely.geometry as geom
import numpy as np
from math import floor

from road_crossing_consts import *


def circum_circle_radius(A, B, C):
    '''
    Return the radius value of circumcircle for triange defined by three points A, B, C.

    Parameters
    ----------
    A: shapely.geometry.Point
        First point defining the triangle.
    B: shapely.geometry.Point
        Second point defining the triangle.
    C: shapely.geometry.Point
        Third point defining the triangle.
    '''
    # Calculate lengths of individual sides.
    lenA = B.distance(C)
    lenB = A.distance(C)
    lenC = A.distance(B)

    if lenA > 0 and lenB > 0 and lenC > 0:  # lengths are nonzero and positive
        divider = np.sqrt(np.fabs((lenA+lenB+lenC) * (lenB+lenC-lenA) * (lenA+lenC-lenB) * (lenA+lenB-lenC)))
        if divider == 0:
            return 10000
        else:
            return float((lenA*lenB*lenC)/divider)
    else:
        return 10000


def get_average_radius(road_network):
    '''
    Returns radius for every road segment for all roads in road network.

    Parameters
    ----------
    road_network: shapely.geometry.MultiLineString
        All the roads in our area represented as MultiLineString.
    '''
    names = ['radius', 'length', 'coords', 'curvature_level', 'curvature']
    formats = ['f8', 'f8', '(2,2)f8', 'i1', 'f8']
    curve_type = dict(names=names, formats=formats)
    segments = []

    for road in list(road_network.geoms):
        radius = []
        road_coords = list(road.coords)
        if len(road_coords) < 3:  # we are not able to calculate circumcircle's radius
            avg_radius = 10000
            seg_len = geom.Point(road_coords[0]).distance(geom.Point(road_coords[1]))
            segments.append(np.array([tuple([avg_radius, seg_len, (road_coords[0], road_coords[1]), 0, 0])], dtype=curve_type))
            continue
        # Create individual circumcircles and calculate theirs radii
        for i in range(len(road_coords)-2):
            radius.append(circum_circle_radius(geom.Point(road_coords[i]), geom.Point(road_coords[i+1]), geom.Point(road_coords[i+2])))
        # Set radii for corresponding road segments
        road_segments = []
        for i in range(len(road_coords)-1):
            if i == 0:
                avg_radius = radius[0]
            elif i == len(road_coords)-2:
                avg_radius = radius[-1]
            else:
                avg_radius = min(radius[i-1:i+1])
            seg_len = geom.Point(road_coords[i]).distance(geom.Point(road_coords[i+1]))
            road_segments.append(tuple([avg_radius, seg_len, (road_coords[i], road_coords[i+1]), 0, 0]))
        segments.append(np.array(road_segments, dtype=curve_type))

    return segments


def rank_segments_curve(segments, junctions):
    '''
    Set curvature level for each road segment. Segments with junction node are automaticaly highest level.

    Parameters
    ----------
    segments: list
        List of dictionaries describing individual road segments.
    junctions: list
        List of shapely.geometry.Point where are junctions in our road network.
    '''
    weights = np.array([LEVEL_0_WEIGHT, LEVEL_1_WEIGHT, LEVEL_2_WEIGHT, LEVEL_3_WEIGHT, LEVEL_4_WEIGHT, LEVEL_5_WEIGHT])
    junctions_coords = [(junction.x, junction.y) for junction in junctions]

    for road in segments:
        # Check if x and y coordinates of at least one point are in junctions coords
        road['curvature_level'][np.amin(np.isin(road['coords'][:], junctions_coords)[:,0], axis=1)+np.amin(np.isin(road['coords'][:], junctions_coords)[:,1], axis=1)] = 5
        # If radius lower then treshold and curvature level not yet given
        road['curvature_level'][(road['radius'] < LEVEL_4_MAX_RADIUS) & (road['curvature_level'] == 0)] = 4
        road['curvature_level'][(road['radius'] < LEVEL_3_MAX_RADIUS) & (road['curvature_level'] == 0)] = 3
        road['curvature_level'][(road['radius'] < LEVEL_2_MAX_RADIUS) & (road['curvature_level'] == 0)] = 2
        road['curvature_level'][(road['radius'] < LEVEL_1_MAX_RADIUS) & (road['curvature_level'] == 0)] = 1

        road['curvature'] = road['length'] * weights[road['curvature_level']]


def road_cost_for_curve(segments, exploration_limit = 100):
    '''
    Returns segments sorted into 15 levels based on theirs suitability for road traversation.

    Parameters
    ----------
    segments: list
        List of dictionaries describing individual road segments.
    exploration_limit: int, float
        Determines what distance in meters in both direction of road we will explore. Base value -> 100.
    '''
    ranked_segments = []  # resolution -> 15 levels
    count = 0

    for road in segments:
        for i in range(len(road)):
            count += 1
            seg_value = road[i]["curvature"]
            dist1 = road[i]["length"]/2
            for j in range(i, -1, -1):
                seg_value += road[j]["curvature"]
                dist1 += road[j]["length"]
                if dist1 >= exploration_limit:
                    break
            dist2 = road[i]["length"]/2
            for j in range(i, len(road)):
                seg_value += road[j]["curvature"]
                dist2 += road[j]["length"]
                if dist2 >= exploration_limit:
                    break
            seg_value /= (dist1+dist2)
            ranked_segments.append((geom.LineString(road[i]['coords']), (ROAD_CURVATURE_RANKS-1) if seg_value >= (ROAD_CURVATURE_RANKS-1) else floor(seg_value)))
    print("INFO: Processed {} road segments.".format(count))
    return ranked_segments
