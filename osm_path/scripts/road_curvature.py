import shapely.geometry as geom
import numpy as np
from math import floor


def circum_circle_radius(A: geom.Point, B: geom.Point, C: geom.Point):
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


def get_average_radius(road_network: geom.MultiLineString):
    '''
    Returns radius for every road segment for all roads in road network.

    Parameters
    ----------
    road_network: shapely.geometry.MultiLineString
        All the roads in our area represented as MultiLineString.
    '''
    segments = []

    for road in list(road_network.geoms):
        radius = []
        road_coords = list(road.coords)
        if len(road_coords) < 3:  # we are not able to calculate circumcircle's radius
            avg_radius = 10000
            seg_len = geom.Point(road_coords[0]).distance(geom.Point(road_coords[1]))
            segments.append([dict((("radius", avg_radius), ("length", seg_len), ("coords", (road_coords[0], road_coords[1]))))])
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
            road_segments.append(dict((("radius", avg_radius), ("length", seg_len), ("coords", (road_coords[i], road_coords[i+1])))))
        segments.append(road_segments)

    return segments


def rank_segments_curve(segments: list, junctions: list):
    '''
    Set curvature level for each road segment. Segments with junction node are automaticaly highest level.

    Parameters
    ----------
    segments: list
        List of dictionaries describing individual road segments.
    junctions: list
        List of shapely.geometry.Point where are junctions in our road network.
    '''
    level_0_weight = 0.75
    level_1_max_radius = 220
    level_1_weight = 3
    level_2_max_radius = 150
    level_2_weight = 6
    level_3_max_radius = 90
    level_3_weight = 10
    level_4_max_radius = 40
    level_4_weight = 20
    level_5_weight = 30

    junctions_coords = [(junction.x, junction.y) for junction in junctions]

    for road in segments:
        for segment in road:
            if segment["coords"][0] in junctions_coords or segment["coords"][1] in junctions_coords:
                segment['curvature_level'] = 5
                segment['curvature'] = segment['length'] * level_5_weight
                segment['curvature'] = segment['curvature'] if segment['curvature'] > 400 else 400
            elif segment['radius'] < level_4_max_radius:
                segment['curvature_level'] = 4
                segment['curvature'] = segment['length'] * level_4_weight
            elif segment['radius'] < level_3_max_radius:
                segment['curvature_level'] = 3
                segment['curvature'] =  segment['length'] * level_3_weight
            elif segment['radius'] < level_2_max_radius:
                segment['curvature_level'] = 2
                segment['curvature'] =  segment['length'] * level_2_weight
            elif segment['radius'] < level_1_max_radius:
                segment['curvature_level'] = 1
                segment['curvature'] =  segment['length'] * level_1_weight
            else:
                segment['curvature_level'] = 0
                segment['curvature'] =  segment["length"] * level_0_weight


def road_cost_for_curve(segments: list, exploration_limit: int = 100):
    '''
    Returns segments sorted into 15 levels based on theirs suitability for road traversation.

    Parameters
    ----------
    segments: list
        List of dictionaries describing individual road segments.
    exploration_limit: int, float
        Determines what distance in meters in both direction of road we will explore. Base value -> 100.
    '''
    ranked_segments = [[] for i in range(15)]  # resolution -> 15 levels
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
            ranked_segments[14 if seg_value >= 14 else floor(seg_value)].append(road[i])
    print("INFO: Processed {} road segments.".format(count))
    return ranked_segments
