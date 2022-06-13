## What is this
A package for navigation of a robot using the Emlid Reach M+ GPS module.

## How to use this package
1. Run gps.launch (with the desired parameters, most notably the gpx_file param with the .gpx file name, which contains the points we want the robot to go through).
2. Prepare robot for autonomous driving.
3. Run follower.launch. The robot stops after reaching the goal (unless gps.launch -> repeat=True) or after the follower.launch is stopped.

## What other commands to run with this
**rosrun nifti_teleop priority_wrapper.py**

## Launch files description
#### /launch/gps.launch

Publishes the path topic which is used by the path follower to autonomously drive the robot.

- Nodes:
**mag_field_to_azimuth**
Takes IMU magnetometer measurings and calculates the heading of the robot in relation to north.
**llh_to_fix**  
Takes Emlid Reach M+ .LLH messages through serial port and publishes the data as a fix message.
**gps_to_path** 
Creates waypoints from provided .gpx file. Takes UTM position of the robot, azimuth and the waypoints and publishes a path message for the robot to follow.

- Published topics:
**mag_azimuth** 
**fix** 
**path** 

- Parameters:
**num_published_path_points **
Number of points from the path that are published for the robot to navigate itself by. The path follower prefers (i.e. follows) points with a higher index. If set too high, the path follower might drive straight to the goal (= the point with the highest index).
**gpx_file **
Name of the file with the coordinates we want the robot to drive through (in order).
**circular** 
Adds waypoints connecting the goal point with the start point.
**reversed_path** 
Reverses the order of the waypoints. The robot will go through them in a reversed order.
**path_points_density** 
In meters. Distance between a pair of path points.
**repeat** 
Makes the robot drive through the same waypoints indefinitely.
**start_from_nearest_point** 
Makes the robot not start in the start (=first) point, but instead in the nearest point.

#### /launch/follower.launch

From NAEX package. Makes the robot follow a path.
