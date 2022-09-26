# Unhost paths
https://docs.google.com/document/d/1EATuN6nUMDcxek5O1ZXXLCy_q1Qzpz020aAbURKAkIs/edit
https://en.mapy.cz/s/nunupunepo

# TL;DR Most important stuff is here.

### Branches:
 - Use **python2** branch on all robots even husky.

### Use path_planner_igraph.launch as in this example:<br /><br />
**roslaunch gps_to_path path_planner_igraph.launch <br /> gpx_assignment:=~/gps_ws/src/gps-navigation/gps_to_path/data/unhost.gpx <br /> run_ntrip_relay:=true <br /> robot_base_frame:=base_link <br /> use_osm:=true <br /> flip:=false <br /> trav_obstacle_field:=cost <br /> trav_frame:=os_lidar  <br /> max_height:=1. <br /> min_height:=0.05 <br /> <br />**
 - Initialization takes some time because of use_osm:=true. If you want it to be much faster, set use_osm:=false. You lose OSM information, but usually, that should be fine. If the initialization gets **stuck on "Running query" inside terminal** you can set use_osm:=false or try and provide a better internet connection (e.g. ~/lte_on.sh on tradr).
 - **max_height should be around the height of the robot from base frame and min_height should be around the distance from base_frame to ground (only traversability point between these two values IN REGARD TO BASE_FRAME will be used, rest will be filtered out). Working values should be around (min 0.05, max 1.) for TRADR and (min -1., max 0.5) for SPOT (don't know for HUSKY)**
 - flip:=true if you need to reverse the order of waypoints.
 - **trav_obstacle_field:=obstacle AND trav_frame:=os_sensor IF cloud_segmentation/lidar_cloud trav_obstacle_field:=cost AND trav_frame:=os_lidar IF geometric_traversability_raw OR CHECK WITH TOMAS PETRICEK.**
 - TRAVERSABILITY IS NOT VITAL FOR THE PLANNER. It can run without it.
 - The whole traversability processing inside the planner node is in try/except for now. So if it suspiciously seems like the traversability is not doing anything, it could be that there is an repeated error, but it is not shown in the console or elsewhere.
 - If you get any error that crashes the planner and you can't fix it, you can always go back to **gps.launch**, which is the old planner (no traversability or OSM or graph search).
<br />

### Use follower.launch like this:<br /><br />
**roslaunch gps_to_path follower.launch robot:=(ctu-robot/spot)**
 - Don't forget the argument if robot is not ctu-robot!
<br />

### Checklist:
 - Check Reach web UI (try 192.168.2.* IP of robot in browser). Check if there is a fix and if corrections are coming (CORRECTIONS ARE ONLY COMING IF THE PATH PLANNER IS RUNNING).
 - Keep an eye on the terminal of the path planner; there should not be any errors.
 - In rviz check if trav_obstacles_pcd_map and whichever traversability point cloud you use are overlaping. If not, set mirrored_points:=-1.
 - In rviz check if path makes sense (if using OSM a check against osm_pcd topic in rviz). If not, check if gpx_assignment is set correctly.
 - In rviz check if the path is not rotating wildly compared to base_link. IF YES, THAN CALIBRATE COMPASS PARAMETERS. EXPECT THIS TO HAPPEN WITH SPOT.


# What is this package
A package for the navigation of a robot using the Emlid Reach M+ GPS module and an IMU.

# How to use it <br />
1. git clone the source code of this package into a catkin workspace.
2. Build your catkin workspace.
3. (optional) Check that you are receiving fix on your robot (without NTRIP corrections) and that your IMU is working. If you are not receiving fix and would still like to test the package out, in the path planner launch file you are about to run (e.g. **launch/path_planner_igraph.launch**) comment out the section **LLH FROM GPS TO FIX** and comment in the section **FAKE GPS**.
4. Run **path_planner_igraph.launch** with arguments (see example lower).
5. Calibrate the compass by rotating the robot in place, 360 degrees in ideally both directions. **OR** If calibration has been done in the past on the robot, then the parameters have been saved and will be used if parameter **load_from_file** of node **get_mag_shift** is set to True (default).
6. (optional) The *path* topic should now be publishing. The *mag_azimuth* topic should be reporting the orientation of the robot and the gps should be receiving NTRIP. Topics which can be visualized in rviz are *trav_obstacles_pcd_map, osm_pcd, graph_pcd*.
7. Prepare robot for autonomous driving.
8. Run follower.launch. The robot stops after reaching the goal or after the follower is terminated.

# Commands to run with examples
## Path planner launch

There are two possible path planner which can be used.

First is the older and simpler **gps.launch**, which calculates path based on GPS current position (**fix**) and compass orientation (**magnetic_azimuth**).

Second is the newer and more advanced **path_planner_igraph.launch**. In addition to the previously mentioned information it uses OSM data and traversability point cloud and calculates optimal path using Djikstra graph search.
  
### gps.launch

**roslaunch gps_to_path gps.launch <br /> gpx_file:=___ <br /> circular_path:=___ <br /> reversed_path:=___ <br /> repeat:=___ <br /> start_from_nearest_point:=___**

- Arguments explained:
    - **gpx_file** <br />
    Name of the file with the coordinates we want the robot to drive through (in order). <br />
    - **circular_path**  <br />
    Adds waypoints connecting the goal point with the start point. <br />
    - **reversed_path**  <br />
    Reverses the order of the waypoints. The robot will go through them in a reversed order. <br />
    - **repeat**  <br />
    Makes the robot drive through the same waypoints indefinitely. <br />
    - **start_from_nearest_point**  <br />
    Makes the robot not start in the start (=first) point, but instead in the nearest point.

- Other parameters inside the launch file explained:
    - **num_published_path_points** <br />
    Number of points from the path that are published for the robot to navigate itself by. The path follower prefers (i.e. follows) points with a higher index. If set too high, the 
    path follower might drive straight to the goal (= the point with the highest index). <br />
    - **path_points_density**  <br />
    In meters. Distance between a pair of path points. <br />

- For example:

    **roslaunch gps_to_path gps.launch gpx_file:=~/gps_ws/src/gps-navigation/gps_to_path/data/cimicky.gpx circular_path:=False reversed_path:=False repeat:=False start_from_nearest_point:=True**

    Another way to launch this file is to first edit the default arguments inside gps.launch and then simply run the command:

    **roslaunch gps_to_path gps.launch**

### path_planner_igraph.launch

**roslaunch gps_to_path path_planner_igraph.launch <br /> gpx_assignment:=___ <br /> run_ntrip_relay:=___ <br /> robot_base_frame:=___ <br /> use_osm:=___ <br /> flip:=___ <br /> trav_obstacle_field:=___ <br /> trav_frame:=___ <br /> max_height:=___ <br /> min_height:=___ <br />**

- Arguments explained:
    - **gpx_assignment** <br />
    Name of the file with the coordinates we want the robot to drive through (in order). <br />
    - **run_ntrip_relay**  <br />
    Run service which supplies the GPS sensor with NTRIP corrections thus improving quality of the produced fix. <br />
    - **robot_base_frame**  <br />
    Base frame of the robot to be used in the planner and in the processing of traversability pointcloud. <br />
    - **use_osm**  <br />
    Use OSM data in the planner. If 'false', the planner initialization is **much faster** and **does not require internet connection**.<br />
    - **flip**  <br />
    Reverse the order of waypoints generated from the gpx_assignment file.<br />
    - **trav_obstacle_field**  <br />
    Name of the field in which the traversability point cloud contains the information whether a point is traversable or not (hint: combinations used in the past are "cloud_segmentation/lidar_cloud" topic and "**obstacle**" trav_obstacle_field and "geometric_traversability_raw" topic with "**cost**".).<br />
    - **trav_frame**  <br />
    Name of the frame in which the traversability point cloud is published. os_lidar fo geometric_traversability_raw and os_sensor for cloud_segmentation/lidar_cloud.<br />
    - **max_height**, **min_height**  <br />
    In meters. Parameters used for filtering untraversable points from the traversability point cloud (e.g. points too low or too high from the robot, related to base_link). Working values should be around (min 0.05, max 1.) for TRADR and (min -1., max 0.5) for SPOT (don't know for HUSKY). <br />

- Other parameters inside the launch file explained:
    - **goal_reached_distance** <br />
    In meters. Maximum distance between robot and goal point for the goal point to be considered reached. <br />
    - **publish_pcd**  <br />
    Allows visualisation of OSM data in rviz. <br />
    - **trav_max_dist**  <br />
    In meters. Maximum distance at which points from traversability point cloud are used. Points further than this distance are ignored. <br />
    - **trav_radius**  <br />
    In meters. Maximum distance from a graph vertex at which untraversable points from point cloud are used to increase the cost of travelling to or from this vertex. <br />
    - **untrav_point_cost**  <br />
    The increase in cost of travelling to or from a graph vertex per each untraversable point from the traversability point cloud. <br />
    - **long_term_memory_longevity**  <br />
    Increasing this parameter causes the costs of vertices outside of trav_max_distance from robot to decay slower (at 1 they do not decay at all). <br />
    - **short_term_memory_longevity**  <br />
    Increasing this parameter causes the costs of vertices inside of trav_max_distance from robot to change slower (i.e. slower reaction to new close untraversable objects). <br />
    - **trav_inheritance_radius**  <br />
   In meters. When a waypoint is reached and another one is taken as current goal, the old graph is retired and a new graph is used. These two graphs usually have a major overlap. This parameter sets the maximum distance for a new graph vortex to inherit cost of a nearby old graph vortex. <br />
    - **max_goal_untrav_cost**  <br />
    When the cost of the graph vortex representing the current goal point exceeds this value (i.e. the goal is untraversable thus unreachable), then the goal is moved one vertex closer along the gas pipeline. <br />
    - **max_subgraph_time**  <br />
    In seconds. If the current goal point has not been reached in this time, the next goal point is selected.<br />
    - **osm_use_solitary_nodes**  <br />
    Use solitary OSM nodes, which usually represent objects such as trees, benches, signs, etc. Speeds up initialization of the planner when set to 'false'. <br />
    - **max_dist**, **min_dist**  <br />
    In meters. Parameters used for filtering untraversable points from the traversability point cloud (e.g. points too close from the robot frame perspective are filtered out). <br />
    - **max_age**  <br />
    In seconds. Maximum age of traversability point cloud message to be used (in case the traversability pcd msg comes at a higher frequency than we can process it at). <br />

- For example:

    **roslaunch gps_to_path path_planner_igraph.launch <br /> gpx_assignment:=~/gps_ws/src/gps-navigation/gps_to_path/data/cimicky.gpx <br /> run_ntrip_relay:=true <br /> robot_base_frame:=base_link <br /> use_osm:=true <br /> flip:=false <br /> trav_obstacle_field:=cost <br />  trav_frame:=os_lidar <br /> max_height:=0.5 <br /> min_height:=-1. <br />**
    Another way to launch this file is to first edit the default arguments inside path_planner_igraph.launch and then simply run the command:

    **roslaunch gps_to_path path_planner_igraph.launch**

    And change any of the other parameters mentioned higher inside the launch file.

## (optional) ICP
**getIcp**

## (optional if running **gps.launch**) OSM data to PCD (osm visualisation)
**roslaunch gps_to_path osm.launch**<br />
Also has a parameter points_density -- set this inside the osm.launch file (set lower for faster performance -- might be necessary for long paths).<br />
The topic to which the point cloud is published is **osm_pcd**.

## (optional) Path follower
**roslaunch gps_to_path follower.launch<br /> robot:=___ <br /> cmd_vel:=___ <br /> max_speed:=___ <br /> allow_backward:=___ <br />**

- Arguments explained:
    - **robot** <br />
    Name of the robot - **IMPORTANT IF ROBOT IS NOT TRADR**. <br />
    - **cmd_vel** <br />
    Topic to which to publish the velocity commands. <br />
    - **max_speed**  <br />
    Maximum forward speed. <br />
    - **allow_backward**  <br />
    Allow the robot to drive backwards. <br />

- For example:

    **roslaunch gps_to_path follower.launch<br /> robot:=spot <br /> cmd_vel:=nav/cmd_vel <br /> max_speed:=0.5 <br /> allow_backward:=false <br />**
    
# Debugging
(Any gps.launch mentions work with path_planner_igraph.launch as well.)
1. **"I run gps.launch but path is not being published."**
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

# Detailed descriptions

1. Nodes
- **get_mag_shift** <br />
Measures for extremes of /imu/mag x,y and z values for n seconds. Publishes the mean x,y and z.
- **mag_field_to_azimuth** <br />
Takes IMU magnetometer measurings and calculates the heading of the robot in relation to north. <br />
- **llh_to_fix**   <br />
Takes Emlid Reach M+ .LLH messages through serial port and publishes the data as a fix message. <br />
- **gps_to_path**  <br />
Creates waypoints from provided .gpx file. Takes UTM position of the robot, azimuth and the waypoints and publishes a path message for the robot to follow. <br />
