INIT_WIDTH = 1          # In meters. Points further than INIT_WIDTH from the desired trajectory are penalized.

DENSITY = 1             # In meters. How dense is the graph (distance between neighboring nodes).

GOAL_BASE_DIST = 30     # In meters. Desired distance between start and goal points in a sub-graph.
                        # If there is an obstacle in the way, the distance is modified accordingly.

INCREASE = 50           # In meters. How much the range of the graph increases to each side
                        # with further iterations. (The actual range increases by 2*INCREASE with each iter).
                        
MAX_RANGE = 300         # In meters. Maximum range of the area that will be searched.

MAX_EFFECTIVE_RANGE = 50000 #  CURRENTLY NOT BEING USED.

MAX_DIST_LOSS = 10      # In meters. Any points furter than MAX_DIST_LOSS from the desired trajectory will
                        # be penalized as if they were only at MAX_DIST_LOSS distance.

MAX_COST_PER_METER = 10 # If the INITIAL solution of a sub-graph has a total cost of more than
                        # MAX_COST_PER_METER * meters_between_start_and_goal, increase graph range and try again.
                        # (((If the graph has found a solution, but it is too expensive, we have it try once more
                        # in an extended range. Important for crossing roads.)))

ROAD_LOSS = 1000        # Penalization of an edge with at least one vertex on a road.
                        # (((Makes the graph search cross roads across footways - those are not considered
                        # as roads - or at least in the shortest manner possible.)))

NO_FOOTWAY_LOSS = 100   # Penalization of an edge at least MAX_DIST_LOSS far and with neither vertex on a footway.
                        # (((The graph search will prefer walking along footways ONCE IT IS FAR from the
                        # desired trajectory anyway.)))
                    
MAX_DETOUR_COUNTER = 3  #  CURRENTLY NOT BEING USED.

if INIT_WIDTH < 1:
    INIT_WIDTH = 1
    print("Parameter INIT_WIDTH set to 1 (minimum).")