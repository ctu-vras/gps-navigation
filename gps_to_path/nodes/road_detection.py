import numpy as np
import utm
import shapely.geometry as geom
from matplotlib import pyplot as plt
from matplotlib import patches as ptch
from math import ceil

highway_tags = ["primary", "secondary", "tertiary", "unclassified", "residential", "primary_link", "secondary_link", \
                "tertiary_link", "service", "track", "cycleway", "busway", "road", "living_street"]
highway_tags.remove("cycleway")
highway_tags.remove("track")
#highway_tags = ["primary", "secondary"]


def get_roads(data):
    '''
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
        if way.tags.get("highway") in highway_tags:
            road_nodes = []
            for node in way.nodes:
                road_nodes.append((node.id, float(node.lat), float(node.lon)))
            highway_nodes.append((road_nodes, way.tags.get("highway")))
    return highway_nodes


def get_crossings(data):
    '''
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
        if node.tags.get("highway") == "crossing" or node.tags.get("footway") == "crossing":
            crossings.append((node.id, float(node.lat), float(node.lon)))
    return crossings


def gps_to_utm(data, withID = True):
    '''
    Transforms GPS degrees coordinates into UTM coordinate system.

    Parameters
    ----------
    data: list of tuples
        GPS data in degree coordinate system.
    withID: bool
        Is ID present in tuple. Base value -> True.
    '''
    data = [utm.from_latlon(node[1 if withID else 0], node[2 if withID else 1]) for node in data]
    return np.array([[node[0], node[1]] for node in data])


def create_line_for_road(road, withId = True):
    '''
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
    '''
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
    '''
    Returns list of shapely.geometry.Point of intersecting points in road_network

    Parameters
    ----------
    road_network: shapely.geometry.MultiLineString
        All the roads in our network represented as MultiLineString.
    '''
    intersections = []
    roads = list(road_network.geoms)
    for i in range(len(roads)):
        for j in range(i+1, len(roads)):
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
                    print("DEBUG: unhandled type in find_intersections():", type(inter))
                    #raise TypeError()
    return intersections


def find_junctions(intersections, road_network):
    '''
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
    '''
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
        for i in range(len(coords)-1):
            c1 = coords[i]
            c2 = coords[i+1]

            dist_meters = geom.Point(c1).distance(geom.Point(c2))
            dist = c2[0]-c1[0], c2[1]-c1[1]
            steps = int(ceil(dist_meters/20))  # maximal length between two points 30 meters
            increment = dist[0]/steps, dist[1]/steps

            for j in range(steps):
                new_road.append(tuple([c1[0]+j*increment[0],c1[1]+j*increment[1]]))
        new_road.append(c2)
        new_new_road_network.append(geom.LineString(new_road))
    new_road_network = new_new_road_network

    return geom.MultiLineString(new_road_network)


def road_class_price(roads):
    costs = {"primary": 5, "secondary": 4, "tertiary": 3, "unclassified": 1, "residential": 2, "primary_link": 5, "secondary_link": 4, \
                    "tertiary_link": 3, "service": 2, "track": 1, "cycleway": 1, "busway": 3, "road": 2, "living_street": 2}
    prices = []
    for road in roads:
        prices.append((geom.LineString(gps_to_utm(road[0], True)), costs[road[1]]))
    return prices


def visualize_curves(segments, crossings, grid = True):
    '''
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
    colors = ["blue", "green", "yellow", "orange", "red", "magenta"]
    for road in segments:
        for segment in road:
            x = [node[0] for node in segment["coords"]]
            y = [node[1] for node in segment["coords"]]
            plt.plot(x, y, color=colors[segment["curvature_level"]])
    for crossing in crossings:
        plt.plot(crossing[0], crossing[1], 'o', color='green')
    if grid:
        plt.grid()
    plt.show()


def visualize_curvature_rank(ranked_segments, crossings, grid = True):
    '''
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
    colors = ["blue", "deepskyblue", "aqua", "turquoise", "limegreen", "green", "yellow", "gold",\
              "orange", "peru", "firebrick", "orangered", "deeppink", "magenta", "purple"]
    for rank in ranked_segments:
        for segment in rank:
            x = [node[0] for node in segment["coords"]]
            y = [node[1] for node in segment["coords"]]
            plt.plot(x, y, color=colors[ranked_segments.index(rank)])

    # legend
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
    orangered_patch = ptch.Patch(color="firebrick", label="Level 10")
    red_patch = ptch.Patch(color="orangered", label="Level 11")
    deeppink_patch = ptch.Patch(color="deeppink", label="Level 12")
    magenta_patch = ptch.Patch(color="magenta", label="Level 13")
    purple_patch = ptch.Patch(color="purple", label="Level 14")
    handles = [blue_patch, deepskyblue_patch, aqua_patch, turquoise_patch, limegreen_patch, green_patch,\
              yellow_patch, gold_patch, orange_patch, tomato_patch, orangered_patch, red_patch, deeppink_patch,\
              magenta_patch, purple_patch]
    ax.legend(handles=handles)

    for crossing in crossings:
        plt.plot(crossing[0], crossing[1], 'o', color='green')
    if grid:
        plt.grid()
    plt.show()
