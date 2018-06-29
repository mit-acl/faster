cmake_minimum_required(VERSION 2.8.3)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O3")
add_compile_options(-std=c++11) 
project(tip)


find_package( Eigen3 REQUIRED )
include_directories(${EIGEN3_INCLUDE_DIR})

find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
add_definitions(${PCL_DEFINITIONS})
link_directories(${PCL_LIBRARY_DIRS})

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
	roscpp
	rospy
	std_msgs
	geometry_msgs
	sensor_msgs
	tf
	rosbag
	roslib
	acl_msgs
	sensor_msgs
	pcl_ros
	tf2
	tf2_ros
	tf2_geometry_msgs
	tf2_sensor_msgs
)

add_message_files(
  DIRECTORY msg
  FILES
  TIP.msg
)


generate_messages(DEPENDENCIES std_msgs geometry_msgs)

catkin_package(INCLUDE_DIRS include CATKIN_DEPENDS message_runtime std_msgs geometry_msgs)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(tip src/main.cpp src/tip.cpp src/utils.cpp src/collision-check.cpp)
add_dependencies(tip ${catkin_EXPORTED_TARGETS})
target_link_libraries(tip ${catkin_LIBRARIES})

#add_executable(unit_test src/debug/test_pcl.cpp src/tip.cpp )
#add_dependencies(unit_test ${catkin_EXPORTED_TARGETS})
#target_link_libraries(unit_test ${catkin_LIBRARIES})

