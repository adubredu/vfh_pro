cmake_minimum_required(VERSION 2.8.3)
project(global_planner)
set(ompl_DIR /usr/local/share/ompl/cmake)
set(eigen_INCLUDE_DIR /usr/include/eigen3/Eigen)
set(Eigen3_DIR /usr/lib/cmake/eigen3)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++14)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_generation
  nav_msgs
  ompl
  pcl_ros
  roscpp
  rospy
  sensor_msgs
  std_msgs
  Eigen3 
)


add_service_files(
  FILES
  Goal.srv
  Waypoints.srv
  )

generate_messages(
  DEPENDENCIES
  geometry_msgs#   nav_msgs#   std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES global_planner
#  CATKIN_DEPENDS geometry_msgs message_generation nav_msgs ompl pcl_ros roscpp rospy sensor_msgs std_msgs
#  DEPENDS system_lib
)


include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS} 
  ${eigen_INCLUDE_DIR} 
  /usr/local/share/ompl/
)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(global_planner src/global_planner.cpp)
add_executable(passthrough_filter src/passthrough_filter.cpp)

target_link_libraries(global_planner ${catkin_LIBRARIES} ${OMPL_LIBRARIES} ${OpenCV_LIBS} ${Eigen3_DIR})
target_link_libraries(passthrough_filter ${catkin_LIBRARIES} ${PCL_LIBRARIES})