cmake_minimum_required(VERSION 3.7.2)
project(gps_to_path)

find_package(catkin REQUIRED COMPONENTS message_generation std_msgs)

add_message_files(FILES
  Wgs.msg
  WgsArray.msg
)

generate_messages(DEPENDENCIES std_msgs)

catkin_package(CATKIN_DEPENDS message_runtime std_msgs)
