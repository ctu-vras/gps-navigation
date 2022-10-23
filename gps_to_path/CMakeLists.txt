cmake_minimum_required(VERSION 3.7.2)
project(gps_to_path)

find_package(catkin REQUIRED COMPONENTS geometry_msgs message_generation sensor_msgs std_msgs)

add_message_files(FILES
  LlhFix.msg
  MagShift.msg
  NavSatDetailedStatus.msg
  NavSatFixQuality.msg
  Waypoints.msg
  Wgs.msg
  WgsArray.msg
)

generate_messages(DEPENDENCIES geometry_msgs sensor_msgs std_msgs)

catkin_package(CATKIN_DEPENDS geometry_msgs message_runtime sensor_msgs std_msgs)
