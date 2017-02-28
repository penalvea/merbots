cmake_minimum_required(VERSION 2.8.3)
project(joint_offset)


find_package(catkin REQUIRED COMPONENTS
  mar_robot_arm5e
  roscpp
  std_msgs
  geometry_msgs
  tf
)

catkin_package(

  CATKIN_DEPENDS mar_robot_arm5e roscpp std_msgs geometry_msgs tf

)

include_directories(
  include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS}
)

add_executable(joint_offset_launcher src/joint_offset_launcher.cpp src/joint_offset.cpp)
target_link_libraries( joint_offset_launcher ${catkin_LIBRARIES})
