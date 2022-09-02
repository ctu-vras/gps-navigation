from __future__ import division
from __future__ import absolute_import
import numpy as np
from geodesy import utm
import shapely.geometry as geom
from matplotlib import pyplot as plt
from matplotlib import patches as ptch
from math import ceil

highway_tags = [u"primary", u"secondary", u"tertiary", u"unclassified", u"residential", u"primary_link", u"secondary_link", \
                u"tertiary_link", u"service", u"track", u"cycleway", u"busway", u"road", u"living_street"]
highway_tags.remove(u"cycleway")
highway_tags.remove(u"track")
#highway_tags = ["primary", "secondary"]


def get_roads(data):
    u'''
    Returns list of all crossable road in given map.
    Return value is list of list of tuples, where each list of tuples represents one road.
    Each tuple represents one Node in OSM Way, and its values are Node.id, Node.lat, Node.lon.

    Parameters
    ----------
    data:
        Data returned from Overpass api query.
    '''
    highway_nodes = []
    for way in data.ways:
        if way.tags.get(u"highway") in highway_tags:
            road_nodes = []
            for node in way.nodes:
                road_nodes.append((node.id, float(node.lat), float(node.lon)))
            highway_nodes.append((road_nodes, way.tags.get(u"highway")))
    return highway_nodes


def get_crossings(data):
    u'''
    Returns list of all crossings in given map.
    Return value is list of tuples, where each tuples represents one crossing.
    Tuple represents one Node in OSM Way, and its values are Node.id, Node.lat, Node.lon.

    Parameters
    ----------
    data:
        Data returned from Overpass api query.
    '''
    crossings = []
    for node in data.nodes:
        if node.tags.get(u"highway") == u"crossing" or node.tags.get(u"footway") == u"crossing":
            crossings.append((node.id, float(node.lat), float(node.lon)))
    return crossings


def gps_to_utm(data, withID = True):
    u'''
    Transforms GPS degrees coordinates into UTM coordinate system.

    Parameters
    ----------
    data: list of tuples
        GPS data in degree coordinate system.
    withID: bool
        Is ID present in tuple. Base value -> True.
    '''
    data = [utm.fromLatLong(node[1 if withID else 0], node[2 if withID else 1]) for node in data]
    return np.array([[node.easting, node.northing] for node in data])


def create_line_for_road(road, withId = True):
    u'''
    Creates shapely.geometry.LineString representation for given road coordinates.

    Parameters
    ----------
    road: list of tuples
        Coordinates of road. Two options (ID, x coord, y coord) or (x coord, y coord)
    withID: bool
        Is ID present in tuple. Base value -> True.
    '''
    if withId:
        line = geom.LineString([[node[1], node[2]] for node in road])
    else:
        line = geom.LineString(road)
    return line


def create_road_network(roads, inUTM = True, withID = True):
    u'''
    Constructs road network for given list of individual roads.

    Parameters
    ----------
    roads: list
        List of lists of tuples with coordinates of road nodes.
    inUTM: bool
        Are the coordinates in UTM. Other option means latitude and longitude in degrees. Base value -> True.
    withID: bool
        Is ID present in tuple. Base value -> True.
    '''
    road_network = []
    for road in roads:
        road_network.append(create_line_for_road((road if inUTM else gps_to_utm(road, withID)), withID if inUTM else False))
    return geom.MultiLineString(road_network)


def find_intersections(road_network):
    u'''
    Returns list of shapely.geometry.Point of intersecting points in road_network

    Parameters
    ----------
    road_network: shapely.geometry.MultiLineString
        All the roads in our network represented as MultiLineString.
    '''
    intersections = []
    roads = list(road_network.geoms)
    for i in xrange(len(roads)):
        for j in xrange(i+1, len(roads)):
            inter = roads[i].intersection(roads[j])
            if inter and inter not in intersections:  # intersection exist and we have not found it yet
                # We only deal with Point and MultiPoint
                # TODO: Deal with LineString and MultiLineString? Should not arrise. It does, problem?
                if type(inter) == geom.Point:
                    intersections.append(inter)
                elif type(inter) == geom.MultiPoint:
                    for point in list(inter.geoms):
                        intersections.append(point)
                else:
                    print u"DEBUG: unhandled type in find_intersections():", type(inter)
                    #raise TypeError()
    return intersections


def find_junctions(intersections, road_network):
    u'''
    Returns list of shapely.geometry.Point of junctions found from intersecting points in road_network

    Parameters
    ----------
    intersections: list
        List of shapely.geometry.Point where there are all points of intersections of roads.
    road_network: shapely.geometry.MultiLineString
        All the roads in our network represented as MultiLineString.
    '''
    junctions = []
    counts = dict()
    for road in list(road_network.geoms):
        
        for point in intersections:
            if point.distance(road) != 0:
               continue

            if list(point.coords)[0] in list(road.coords):
                if list(road.coords)[0] == list(point.coords)[0] or list(road.coords)[-1] == list(point.coords)[0]:  # side nodes
                    if list(point.coords)[0] in counts:
                        counts[list(point.coords)[0]] += 1
                        if counts[list(point.coords)[0]] > 2:  # junction must have at least three roads
                            if point not in junctions:
                                junctions.append(point)
                                counts.pop(list(point.coords)[0])
                    else:  # another road in middle of other one means junction
                        counts[list(point.coords)[0]] = 1
                elif point not in junctions:
                    junctions.append(point)
                    if list(point.coords)[0] in counts:
                        counts.pop(list(point.coords)[0])

    return junctions


def combine_road(junctions, intersections, road_network):
    u'''
    Connect roads that can be connected, road will therefore be from junction/map edge to junction/map edge.

    Parameters
    ----------
    juncitons: list
        List of tuples, representing coordinates of all junctions in map area.
    intersections: list
        List of tuples, representing coordinates of all intersecting roads.
    road_network: shapely.geometry.MultiLineString
        All the roads in our network represented as MultiLineString.
    '''
    combining_points = []
    new_road_network = list(road_network.geoms)
    for point in intersections:
        if point not in junctions:
            combining_points.append(point)
    for point in combining_points:
        first_road = []
        for road in new_road_network:
            if list(point.coords)[0] in list(road.coords) and not first_road:
                first_road = road
            elif list(point.coords)[0] in list(road.coords):
                new_road = list(first_road.coords)[::-1] if list(first_road.coords).index(list(point.coords)[0]) == 0 else list(first_road.coords)
                new_road += list(road.coords)[1:] if list(road.coords).index(list(point.coords)[0]) == 0 else list(road.coords)[-2::-1]

                new_road_network.remove(first_road)
                new_road_network.remove(road)
                new_road_network.append(geom.LineString(new_road))

                break

    # Set minimal distance between nodes for better curve detection
    new_new_road_network = []
    for road in new_road_network:
        new_road = []
        coords = list(road.coords)
        for i in xrange(len(coords)-1):
            c1 = coords[i]
            c2 = coords[i+1]

            dist_meters = geom.Point(c1).distance(geom.Point(c2))
            dist = c2[0]-c1[0], c2[1]-c1[1]
            steps = ceil(dist_meters/20)  # maximal length between two points 30 meters
            increment = dist[0]/steps, dist[1]/steps
            for j in xrange(int(steps)):
                new_road.append(tuple([c1[0]+j*increment[0],c1[1]+j*increment[1]]))
        new_road.append(c2)
        new_new_road_network.append(geom.LineString(new_road))
    new_road_network = new_new_road_network

    return geom.MultiLineString(new_road_network)


def road_class_price(roads):
    costs = {u"primary": 5, u"secondary": 4, u"tertiary": 3, u"unclassified": 1, u"residential": 2, u"primary_link": 5, u"secondary_link": 4, \
                    u"tertiary_link": 3, u"service": 2, u"track": 1, u"cycleway": 1, u"busway": 3, u"road": 2, u"living_street": 2}
    prices = []
    for road in roads:
        prices.append((geom.LineString(gps_to_utm(road[0], True)), costs[road[1]]))
    return prices


def visualize_curves(segments, crossings, grid = True):
    u'''
    Show map of processed area, e.g. its road network and crossings. Roads are show in color depending on their curve
    and crossings are green.

    Parameters
    ----------
    segments: list
        Road segments filtered by their curve level.
    crossings: list
        List of shapely.geometry.Point where are crossings in our road network.
    grid: bool
        Show map with grid. Base value -> True.
    '''
    colors = [u"blue", u"green", u"yellow", u"orange", u"red", u"magenta"]
    for road in segments:
        for segment in road:
            x = [node[0] for node in segment[u"coords"]]
            y = [node[1] for node in segment[u"coords"]]
            plt.plot(x, y, color=colors[segment[u"curvature_level"]])
    for crossing in crossings:
        plt.plot(crossing[0], crossing[1], u'o', color=u'green')
    if grid:
        plt.grid()
    plt.show()


def visualize_curvature_rank(ranked_segments, crossings, grid = True):
    u'''
    Show map of processed area, e.g. its road network and crossings. Roads are show in color depending on their traversability
    level and crossings are green.

    Parameters
    ----------
    segments: list
        Road segments filtered by their curve level.
    crossings: list
        List of shapely.geometry.Point where are crossings in our road network.
    grid: bool
        Show map with grid. Base value -> True.
    '''
    fig, ax = plt.subplots()
    colors = [u"blue", u"deepskyblue", u"aqua", u"turquoise", u"limegreen", u"green", u"yellow", u"gold",\
              u"orange", u"peru", u"firebrick", u"orangered", u"deeppink", u"magenta", u"purple"]
    for rank in ranked_segments:
        for segment in rank:
            x = [node[0] for node in segment[u"coords"]]
            y = [node[1] for node in segment[u"coords"]]
            plt.plot(x, y, color=colors[ranked_segments.index(rank)])

    # legend
    blue_patch = ptch.Patch(color=u"blue", label=u"Level 00")
    deepskyblue_patch = ptch.Patch(color=u"deepskyblue", label=u"Level 01")
    aqua_patch = ptch.Patch(color=u"aqua", label=u"Level 02")
    turquoise_patch = ptch.Patch(color=u"turquoise", label=u"Level 03")
    limegreen_patch = ptch.Patch(color=u"limegreen", label=u"Level 04")
    green_patch = ptch.Patch(color=u"green", label=u"Level 05")
    yellow_patch = ptch.Patch(color=u"yellow", label=u"Level 06")
    gold_patch = ptch.Patch(color=u"gold", label=u"Level 07")
    orange_patch = ptch.Patch(color=u"orange", label=u"Level 08")
    tomato_patch = ptch.Patch(color=u"peru", label=u"Level 09")
    orangered_patch = ptch.Patch(color=u"firebrick", label=u"Level 10")
    red_patch = ptch.Patch(color=u"orangered", label=u"Level 11")
    deeppink_patch = ptch.Patch(color=u"deeppink", label=u"Level 12")
    magenta_patch = ptch.Patch(color=u"magenta", label=u"Level 13")
    purple_patch = ptch.Patch(color=u"purple", label=u"Level 14")
    handles = [blue_patch, deepskyblue_patch, aqua_patch, turquoise_patch, limegreen_patch, green_patch,\
              yellow_patch, gold_patch, orange_patch, tomato_patch, orangered_patch, red_patch, deeppink_patch,\
              magenta_patch, purple_patch]
    ax.legend(handles=handles)

    for crossing in crossings:
        plt.plot(crossing[0], crossing[1], u'o', color=u'green')
    if grid:
        plt.grid()
    plt.show()
