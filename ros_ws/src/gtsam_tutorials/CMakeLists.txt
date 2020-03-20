cmake_minimum_required(VERSION 2.8.3)
project(gtsam_tutorials)

find_package(catkin REQUIRED COMPONENTS
  roscpp
)

find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED COMPONENTS
	thread
	system
)
find_package(Eigen3 REQUIRED)
find_package(GTSAMCMakeTools)
find_package(GTSAM REQUIRED)
include_directories(${GTSAM_INCLUDE_DIR})

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS} ${CMAKE_CXX_FLAGS_RELEASE}")

include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
#	${EIGEN3_INCLUDE_DIR}

)

catkin_package(
  #INCLUDE_DIRS include
#  LIBRARIES particle_filter
#  CATKIN_DEPENDS roscpp std_msgs
#  DEPENDS system_lib
)

include_directories(
#  include
#  ${catkin_INCLUDE_DIRS}
)

add_executable(CameraResectioning src/CameraResectioning)
add_executable(creating_fg src/creating_fg)
add_executable(localization src/localization)
add_executable(pose_slam src/pose_slam)

target_link_libraries(CameraResectioning ${catkin_LIBRARIES} ${OpenCV_LIBRARIES}
	${Boost_LIBRARIES}  gtsam)

target_link_libraries(creating_fg ${catkin_LIBRARIES} ${OpenCV_LIBRARIES}
	${Boost_LIBRARIES} gtsam)

target_link_libraries(localization ${catkin_LIBRARIES} ${OpenCV_LIBRARIES}
	${Boost_LIBRARIES}  gtsam)

target_link_libraries(pose_slam ${catkin_LIBRARIES} ${OpenCV_LIBRARIES}
	${Boost_LIBRARIES}  gtsam)



# add_dependencies(landmark_track slam_main_generate_messages_cpp)
