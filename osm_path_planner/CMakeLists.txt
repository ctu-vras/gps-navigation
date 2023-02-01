cmake_minimum_required(VERSION 3.8.2)
project(osm_path_planner)

find_package(catkin REQUIRED COMPONENTS message_generation geometry_msgs std_msgs)

add_message_files(FILES
  Waypoints.msg
  Wgs.msg
  WgsArray.msg
)

generate_messages(DEPENDENCIES
  geometry_msgs
  std_msgs
)

catkin_package(CATKIN_DEPENDS
  message_runtime
  geometry_msgs
  std_msgs 
)


