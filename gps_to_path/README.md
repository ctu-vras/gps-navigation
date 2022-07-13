## What is this
A package for navigation of a robot using the Emlid Reach M+ GPS module.

## How to use this package <br />
0. Obtain means of the magnetometer (topic imu/mag) x,y and z measurements. <br />
1. Run gps.launch (with the desired parameters, most notably the gpx_file param with the .gpx file name, which contains the points we want the robot to go through and the magnetometer parameters). <br />
2. Prepare robot for autonomous driving. <br />
3. Run follower.launch. The robot stops after reaching the goal (unless gps.launch -> repeat=True) or after the follower.launch is stopped.

## What commands to run <br />
**roslaunch gps.launch** <br />
roslaunch nifti_vision_data rec_ugv_min.launch additional_topics:="fix path whole_path mag_azimuth..." <br />
**roslaunch follower.launch cmd_vel:=_ max_speed:=_ allow_backward:=_**

## Launch files description
### /launch/gps.launch

Publishes the path topic which is used by the path follower to autonomously drive the robot.

#### Nodes: <br />
- **mag_field_to_azimuth** <br />
Takes IMU magnetometer measurings and calculates the heading of the robot in relation to north. <br />
- **llh_to_fix**   <br />
Takes Emlid Reach M+ .LLH messages through serial port and publishes the data as a fix message. <br />
- **gps_to_path**  <br />
Creates waypoints from provided .gpx file. Takes UTM position of the robot, azimuth and the waypoints and publishes a path message for the robot to follow. <br />

#### Published topics: <br />
- **mag_azimuth**  <br />
- **fix**  <br />
- **path**  <br />

#### Parameters: <br />
- **num_published_path_points** <br />
Number of points from the path that are published for the robot to navigate itself by. The path follower prefers (i.e. follows) points with a higher index. If set too high, the path follower might drive straight to the goal (= the point with the highest index). <br />
- **gpx_file** <br />
Name of the file with the coordinates we want the robot to drive through (in order). <br />
- **circular**  <br />
Adds waypoints connecting the goal point with the start point. <br />
- **reversed_path**  <br />
Reverses the order of the waypoints. The robot will go through them in a reversed order. <br />
- **path_points_density**  <br />
In meters. Distance between a pair of path points. <br />
- **repeat**  <br />
Makes the robot drive through the same waypoints indefinitely. <br />
- **start_from_nearest_point**  <br />
Makes the robot not start in the start (=first) point, but instead in the nearest point.

### /launch/follower.launch

From NAEX package. Makes the robot follow a path.
