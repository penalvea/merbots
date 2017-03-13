cmake_minimum_required(VERSION 2.8.3)
project(merbots_grasp_srv)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  std_msgs
  message_generation
)


 add_service_files(
   FILES
   grasp_srv.srv
   open_gripper_srv.srv
 )

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp sensor_msgs std_msgs
)
include_directories(
    ${PROJECT_SOURCE_DIR}/include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
    src/grasp.cpp)


add_executable(grasp_srv_server src/grasp_srv_server.cpp)
target_link_libraries(grasp_srv_server
    ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)


add_dependencies(
${PROJECT_NAME}
${PROJECT_NAME}_gencpp
)
add_dependencies(
grasp_srv_server
${PROJECT_NAME}_gencpp
)
