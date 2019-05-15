cmake_minimum_required(VERSION 2.8.3)
project(reverse_priority)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  urdf
  roscpp
  kdl_parser
  message_generation
  cmake_modules
  kdl_conversions
  sensor_msgs
  std_msgs
  tf_conversions
  trajectory_generation
)

add_definitions (-fpermissive -std=c++11)

include_directories(include ${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS})

link_directories(${catkin_LIBRARY_DIRS})


generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  sensor_msgs
)

catkin_package(
  CATKIN_DEPENDS
    geometry_msgs
    urdf
    kdl_parser
    message_runtime
    trajectory_generation
  INCLUDE_DIRS include 
  LIBRARIES ${PROJECT_NAME} 
)

catkin_package(
 INCLUDE_DIRS include
 CATKIN_DEPENDS
 trajectory_generation
)
find_package (Eigen3)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)


add_executable(reverse_priority src/reverse_priority_step.cpp src/main.cpp)
add_dependencies(reverse_priority ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(reverse_priority ${catkin_LIBRARIES})


