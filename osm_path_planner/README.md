## What is this
This ROS package contains a path planner node and other supporting nodes and Python scripts. The planner uses OSM data and possibly also a traversability 
point-cloud to plan a path for a robot from the robot's position to a goal point. The navigation works in the UTM coordinate system, so this package 
depends heavily on some gnss drivers (e.g. https://github.com/ctu-vras/gnss-drivers).

## How to use the path planner
For live planning on a robot use **path_planner_igraph.launch**. For running on a bag file you can use **bag_path_planner_igraph.launch**. The launch files
set a lot of parameters, which are explained below.

In ./launch/ is also included a **visualization.launch** which visualizes many different topics (use rviz), **joy_waypoint.launch** which is used to
tell the planner to move one waypoint forward/backward by using the joystick and **follower.launch** which uses the simple path follower (https://github.com/tpet/naex/blob/master/scripts/path_follower)
to go through the waypoints produced by the planner.

The most basic workflow looks like this:
  1. Roslaunch path_planner_igraph.launch with the parameters you want.
  2. Check that you are receiving the path msg.
  3. (Ideally check everything is in order in rviz.)
  4. Roslaunch follower.launch to make the robot go.


### Following is stuff from the README of the original gps-navigation repository. Some of it might be outdated or not directly relevant to the contents of this package. I have decided to leave it here, as it contains some useful tips and could come in handy when debugging.

#### Launch file
 - roslaunch gps_to_path path_planner_igraph.launch
 - Nodes:
    -  path_planner_igraph - Graph search based planner. Combines information from lidar traversability and OSM data. Main output is waypoints for the robot to follow.
    -  path_waypoints_to_path - Subscribes to waypoints and publishes path, which is used by the path_follower.
    -  path_waypoints_to_path_utm - Same as above but instead of odom frame (gps_odom) the path is published in utm frame (utm).
    -  waypoint_joy - Gamepad control of waypoints.
    -  osm_visualizer - Visualizes OSM data in rviz as point cloud.
 - Arguments & Parameters:
    - **gpx_assignment** <br />
    Name of the file with the coordinates we want the robot to drive through. The file is of the gpx format. Usually these files are located in the gps_to_path/data folder. <br />
       - Value: string path to file
       - Example: "$(find gps_to_path)/data/unhost_final_demo_husky_east_trimmed.gpx"
    - **goal_reached_distance** <br />
    In meters. Maximum distance between robot and goal point for the goal point to be considered reached. <br />
       - Recommended value: 2.
    - **publish_pcd**  <br />
    Currently not used. ~~Allows visualisation of OSM data in rviz.~~ <br />
    - **trav_max_dist**  <br />
    In meters. Maximum distance at which points from traversability point cloud are used. Points further than this distance are ignored. This parameter is not used to filter the traversability point cloud per se, but rather to speed up the planning.<br />
    - **points_absence_radius**  <br />
    In meters. Distance from robot in which graph vertices are penalized if there are no traversability points nearby them. This is supposed to stop the robot from going to nearby unexplored places (e.g. holes in ground).<br />
       - Recommended value: 2.
    - **trav_radius**  <br />
    In meters. Maximum distance from a graph vertex at which untraversable points from point cloud are used to increase the cost of travelling to or from this vertex. <br />
    - **untrav_cost**  <br />
    Multiplier of mean of cost field of traversability points. E.g. if a vortex has two trav points near itself with costs 0.5 and 1.3, the mean is 0.9 and the trav cost applied to the graph is 90 for this given vortex. <br />
       - Recommended value: 100. if using traversability which has cost field in the range [0,2]. Less if you want less strict traversability obeying. Also consider max_traversable_cost.
    - **absence_cost**  <br />
    Related to points_absence_radius. If there are no trav points near a near vortex and there have never been any trav points there in the past, then we treat this vortex as a dangerous unexplored place and we penalize it. Should be higher than max_traversable_cost. <br />
       - Recommended value: Two times untrav_cost.
    - **max_traversable_cost**  <br />
    If a vortex has its traversability cost PLUS absence cost larger than this parameter, than it is "forbidden" and its cost is bumped up to infinity. Robot effectively cannot plan through infinity-cost vertices. <br />
       - Recommended value: Set so that once we reach a trav_mean (mean of cost field of traversability points near a vortex) which we consider untraversable (such that we do not want the robot to go to such a point) then max_traversable_cost = trav_mean x untrav_cost.
       - Example: Say we want to forbid a vortex if traversability says its cost mean is 1. or higher. Then set this equal to untrav_cost (i.e. :=100).
    - **long_term_memory_longevity**  <br />
    Increasing this parameter causes the costs of vertices outside of trav_max_distance from robot to decay slower (at 1 they do not decay at all). More precisely the planner runs at around 10 Hz and each time it multiplies all trav costs by this value (e.g. for 0.997 it reduces the values each second to 97 %). <br />
       - Recommended value: Perhaps around 0.997. Then 1 s ~ 97 %, then 1 minute ~ 16.5 %, 5 minutes ~ 0.01 %.
    - **short_term_memory_longevity**  <br />
    Increasing this parameter causes the costs of vertices inside of trav_max_distance from robot to change slower (i.e. slower reaction to new close untraversable objects). Similar to above, but only applies to vertices which recieved lidar points in the current cycle. The formula is (p x old_cost) + ((1-p) x new_cost) <br />
       - Recommended value: 0.9. If the robot seems to be reacting slowly to obstacles then perhaps higher.
    - **trav_inheritance_radius**  <br />
   In meters. When a waypoint is reached and another one is taken as current goal, the old graph is retired and a new graph is used. These two graphs usually have a major overlap. This parameter sets the maximum distance for a new graph vortex to inherit cost of a nearby old graph vortex. <br />
       - Recommended value: sqrt(2)/2/(points per meter in graph lattice) (slightly larger than that).
       - Example: For 0.5 m between points in graph use 0.36.
    - **max_goal_untrav_cost**  <br />
    When the trav cost of the graph vortex representing the current goal point exceeds this value (i.e. the goal is untraversable thus unreachable), then the goal is moved one vertex closer along the gas pipeline. <br /> 
       - Recommended value: equal to max_traversable_cost.
    - **max_subgraph_time**  <br />
    In seconds. Multiplier per meter of distance between start and goal waypoints (NOT TAKING INTO ACCOUNT ROBOT POSITION). If goal has not been reached in max_subgraph_time x distance we skip to next waypoint. Starts counting once the planner loads OSM data!<br />
       - Recommended value: Around 30. sounds reasonable.
    - **use_osm**  <br />
    Use data from OSM. Either you have a pickle premade for the given gpx file, or one will be created in initialization phase.<br />
       - Recommended value: true unless something is buggy, then false can help.
    - **osm_use_solitary_nodes**  <br />
    Use solitary OSM nodes, which usually represent objects such as trees, benches, signs, etc. Speeds up initialization of the planner when set to 'false'. <br />
       - Recommended value: false
    - **repeat**  <br />
    After arriving to the last waypoint, start from the beggining. <br />
       - Value: true or false. true has not been tested much.
    - **next_waypoint_proximity_switch**  <br />
    Should not be used. <br />
       - Recommended value: false but should not matter.
    - **min_time_near_goal**  <br />
    Run service which supplies the GPS sensor with NTRIP corrections thus improving quality of the produced fix. <br />
       - Recommended value: true
    - **flip**  <br />
    Reverse order of waypoints. IF TRUE YOU ALSO HAVE TO DELETE THE PICKLED .osm_planner FILE BEFORE RUNNING! <br />
       - Value: true or false.
    - **default_trav_cost**  <br />
    Reverse order of waypoints. IF TRUE YOU ALSO HAVE TO DELETE THE PICKLED .osm_planner FILE BEFORE RUNNING! <br />
       - Recommended value: Half of max_traversable_cost/100 or 0 if it is buggy.
    - **untrav_points_buffer**  <br />
    If a point's trav cost is over max_traversable_cost threshold and is bumped up to infinity, also bump all neighboring points. <br />
       - Value: true or false. Definitely false if the trav reports some point like the robot's antenna as untraversable...
    - **max_height**, **min_height**  <br />
    In meters. Parameters used for filtering untraversable points from the traversability point cloud (e.g. points too low or too high from the robot, related to base_link).
       - Recommended values:
           - TRADR (min -1., max 1.), SPOT (min -1., max 0.5), HUSKY not sure? same as TRADR or depending on where the base_link frame is... Also the min values could maybe be even lower? <br />
    - **max_age**  <br />
    In seconds. Maximum age of traversability point cloud message to be used (in case the traversability pcd msg comes at a higher frequency than we can process it at). <br />
       - Recommended value: 1.
    - **trav_obstacle_field**  <br />
    Name of the field in which the traversability point cloud contains the information whether a point is traversable or not.<br />
       - Recommended value: cost
    - **trav_topic**  <br />
    Name of the frame in which the traversability point cloud is published.<br />
       - Recommended value: traversability
    - **odom_frame**  <br />
    Name of the odometry frame which is used for most transformations.<br />
       - Recommended value: gps_odom
    - **max_relative_path_cost_change**  <br />
    Run service which supplies the GPS sensor with NTRIP corrections thus improving quality of the produced fix. <br />
       - Recommended value: true
    - **heading_diff_cost**  <br />
    Multiplies the absolute value of the angle between the robot's heading and the vortex in radians. Penalizes rotations of the robot. ONLY APPLIES TO VERTICES IN A SMALL RADIUS AROUND ROBOT.<br />
       - Recommended value: Around 10. seems reasonable but has not been tested properly.
       - Example: If 10., then the vortex directly behind robot will have penalty ~16 to its cost.

 - **balancing the graph costs** - there was not much done in this regard, so you will probably have to adjust some numbers, following are the parts of the total cost and their range and parameter which sets that range:
1. Traversability cost - 0-200 - parametr untrav_cost
2. Absence cost - 0 or 200 - parametr absence_cost
3. Forbidden cost - 0 or infinity - if sum of previous two is larger than max_traversable_cost than inf
4. Road cost - 180-220 - ROAD_LOSS in graph_serach_params.py
5. No footway cost - 0 or 10 - graph_search_params.py - NO_FOOTWAY_LOSS in graph_serach_params.py
6. Barrier cost - 0 or 10000 - graph_search_params.py - BARRIER_LOSS in graph_serach_params.py

### Branches:
 - Use **master** branch on all robots.

### Use path_planner_igraph.launch as in this example:<br /><br />
**roslaunch gps_to_path path_planner_igraph.launch <br /> gpx_assignment:=~/gps_ws/src/gps-navigation/gps_to_path/data/unhost.gpx <br /> run_ntrip_relay:=true <br /> robot_base_frame:=base_link <br /> use_osm:=true <br /> flip:=false <br /> trav_obstacle_field:=cost <br /> trav_frame:=os_lidar  <br /> max_height:=1. <br /> min_height:=0.05 <br /> <br />**
 - Initialization takes some time because of use_osm:=true. If you want it to be much faster, set use_osm:=false. You lose OSM information, but usually, that should be fine. If the initialization gets **stuck on "Running query" inside terminal** you can set use_osm:=false or try and provide a better internet connection (e.g. ~/lte_on.sh on tradr).
 - **max_height should be around the height of the robot from base frame and min_height should be around the distance from base_frame to ground (only traversability point between these two values IN REGARD TO BASE_FRAME will be used, rest will be filtered out). Working values should be around (min 0.05, max 1.) for TRADR and (min -1., max 0.5) for SPOT (don't know for HUSKY)**
 - flip:=true if you need to reverse the order of waypoints.
 - **trav_obstacle_field:=obstacle and trav_topic:=cloud_segmentation/lidar_cloud OR trav_obstacle_field:=cost AND trav_topic:=geometric_traversability_raw.**
 - TRAVERSABILITY IS NOT VITAL FOR THE PLANNER. It can run without it.
<br />

### Use follower.launch like this:<br /><br />
**roslaunch gps_to_path follower.launch robot:=ctu-robot/husky-robot/spot-3**
 - Don't forget the argument!
<br />

### Checklist:
 - Check Reach web UI (try 192.168.2.* IP of robot in browser). Check if there is a fix and if corrections are coming (CORRECTIONS ARE ONLY COMING IF THE PATH PLANNER IS RUNNING).
 - Keep an eye on the terminal of the path planner; there should not be any errors.
 - In rviz check if trav_obstacles_pcd_map and whichever traversability point cloud you use are overlaping. If not, set mirrored_points:=-1.
 - In rviz check if path makes sense (if using OSM a check against osm_pcd topic in rviz). If not, check if gpx_assignment is set correctly.
 - In rviz check if the path is not rotating wildly compared to base_link. IF YES, THAN CALIBRATE COMPASS PARAMETERS. EXPECT THIS TO HAPPEN WITH SPOT.

# Debugging
(Any gps.launch mentions work with path_planner_igraph.launch as well.)
1. **"I run path_planner_igraph.launch but path is not being published."**
- Check if **fix topic** is being published. If not see point 2.
2. **"Fix is not being published."**
- In your phone open the Reachview3 application or in your browser open the IP address to which Reach is connected (you can see the IP in Reachview or you can figure it out from nmap).
- In the interface check if the GPS sees any satellites. If not the you are either in a spot with **no signal** (e.g. inside a building) or the **GPS has overheated**.
3. **"Reachview3 doesn't show the Reach I am looking for."**
- You are connected to the **wrong network** or the **GPS has overheated**. There could also be a network issue on the robot's side.
4. **"I am getting some weird response/error from the NTRIP server."** (you would see this as a message in the terminal running gps.launch)
 - Check **NTRIP settings** through Reachview or Reach's web interface. The address is **ntrip.pecny.cz**, port is **2101** and the NTRIP mountpoint should be filled as **CRAK00CZE0**. If everything checks out try to turn it off and on.
5. **"The fix has a larger covariance than expected."**
 - NTRIP is probably not working (check in Reachview/web UI). You have to be running gps.launch, or something else that contains the **ntrip relay**. The robot needs access to the **internet**. Also see 4.
6. **The path is being published, but it is clearly wrong/offset.**
 - Check if your fix has decent covariance (or check reported position in Reach web UI).
 - Check if you are not using the **fake fix node** inside of the gps.launch. Check if the **gpx_file** argument of gps.launch is set and set correctly.
7. **The path is being published, but it is jumping around like crazy.**
 - Check the mag_azimuth topic. If its value is jumping a lot, then you have probably forgot to do the **calibration rotation** of the robot after running gps.launch. The rotation has to be done during the **30 seconds after running gps.launch**.
8. **"The robot has been driving autonomously but has suddenly and prematurely stopped."**
 - Check if fix is coming - if not see 2 (probably overheated GPS).
 - Check terminal with follower.launch - does the robot think it has arrived to the goal? Check in rviz what the published path looks like. Could be poor fix quality or some issue with the published path.
9. **"The robot is driving autonomously but is rapidly switching between going forward and backward or acting crazy."**
 - Could be an issue with the assigned path being **linear** and the argument **circular_path** of gps.launch being set to True. The path follower always calculates the path from the nearest point to his current location and this point can be switching rapidly if the path in both directions leads through almost exactly the same place. Change either the parameter or the assignment.
 - Should not be problem if using path_planner_igraph.launch.
 - Also could be problem with **the path crossing itself**. Similar issue to above. You can manually drive through the crossing.
