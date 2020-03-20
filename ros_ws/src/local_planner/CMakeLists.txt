cmake_minimum_required(VERSION 2.8.3)
project(local_planner)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  pcl_ros
  roscpp
  rospy
  std_msgs
  laser_geometry
  tf
)
find_package(PCL REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES local_planner
#  CATKIN_DEPENDS geometry_msgs nav_msgs pcl_ros roscpp rospy std_msgs tf
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  /usr/local/include
  /usr/include
)


link_directories(
	/usr/local/lib
	/usr/lib
	${PCL_LIBRARY_DIRS}
)
add_definitions(${PCL_DEFINITIONS})

add_executable(path_follower src/path_follower.cpp)
add_executable(local_planner src/local_planner.cpp)
add_executable(laserscan_to_pointcloud src/laserscan_to_pointcloud.cpp)

target_link_libraries(path_follower ${catkin_LIBRARIES})
target_link_libraries(local_planner ${catkin_LIBRARIES} ${PCL_LIBRARIES})
target_link_libraries(laserscan_to_pointcloud ${catkin_LIBRARIES} )