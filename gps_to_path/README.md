# Paths for experiment
https://en.mapy.cz/s/bumegejeja

# What is this package
A package for the navigation of a robot using the Emlid Reach M+ GPS module and an IMU.

# How to use it <br />
1. git clone the source code of this package into a catkin workspace.
2. Build your catkin workspace.
3. (optional) Check that you are receiving fix on your robot (without NTRIP corrections) and that your IMU is working. If you are not receiving fix and would still like to test the package out, in **launch/gps.launch** comment out the section **LLH FROM GPS TO FIX** and comment in the section **FAKE GPS**.
4. Run **gps.launch** with arguments (see example lower).
5. Calibrate the compass by rotating the robot in place, 360 degrees in ideally both directions.
6. (optional) The *path* topic should now be being published. The *mag_azimuth* topic should be reporting the orientation of the robot and the gps should be receiving NTRIP.
7. Prepare robot for autonomous driving.
8. Run follower.launch. The robot stops after reaching the goal (unless gps.launch -> repeat=True) or after the follower.launch is stopped.

# Commands to run with examples
## Gps launch
   
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

- For example:

    **roslaunch gps_to_path gps.launch gpx_file:=~/gps_ws/src/gps-navigation/gps_to_path/data/cimicky.gpx circular_path:=False reversed_path:=False repeat:=False start_from_nearest_point:=True**

    Another way to launch this file is to first edit the default arguments inside gps.launch and then simply run the command:

    **roslaunch gps_to_path gps.launch**

## (optional) Recording (TRADR robot)
**roslaunch nifti_vision_data rec_ugv_min.launch additional_topics:="fix path whole_path mag_azimuth mag_shift"**

## (optional) ICP
**getIcp**

## (optional) OSM data to PCD (osm visualisation)
**roslaunch gps_to_path osm.launch**<br />
Also has a parameter points_density -- set this inside the osm.launch file (set lower for faster performance -- might be necessary for long paths).<br />
The topic to which the pointcloud is published is **osm_pcd**.

## (optional) Path follower
**roslaunch gps_to_path follower.launch<br /> cmd_vel:=___ <br /> max_speed:=___ <br /> allow_backward:=___ <br />**

- Arguments explained:
    - **cmd_vel** <br />
    Topic to which to publish the velocity commands. <br />
    - **max_speed**  <br />
    Maximum forward speed. <br />
    - **allow_backward**  <br />
    Allow the robot to drive backwards. <br />

- For example:

    **roslaunch gps_to_path follower.launch<br /> cmd_vel:=cmd_vel <br /> max_speed:=0.5 <br /> allow_backward:=False <br />**
    
# Debugging
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

2. Other parameters <br />
- **num_published_path_points** <br />
Number of points from the path that are published for the robot to navigate itself by. The path follower prefers (i.e. follows) points with a higher index. If set too high, the path follower might drive straight to the goal (= the point with the highest index). <br />
- **path_points_density**  <br />
In meters. Distance between a pair of path points. <br />
