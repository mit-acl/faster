cmake_minimum_required(VERSION 2.8.3)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O3")
project(acl_planning)


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
	acl_system
	sensor_msgs
	pcl_ros
	)

# generate_messages(
#   DEPENDENCIES
#   std_msgs  # Or other packages containing msgs
#   geometry_msgs
# )

catkin_package(
	INCLUDE_DIRS include 
	# CATKIN_DEPENDS message_runtime std_msgs geometry_msgs
	# CATKIN_DEPENDS message_runtime std_msgs geometry_msgs
)

include_directories(
	include
    ${catkin_INCLUDE_DIRS}
    )

add_executable(reactive_planner src/main.cpp src/rp.cpp )
add_dependencies(reactive_planner ${catkin_EXPORTED_TARGETS})
target_link_libraries(reactive_planner ${catkin_LIBRARIES})

add_executable(unit_test src/test_pcl.cpp src/rp.cpp )
add_dependencies(unit_test ${catkin_EXPORTED_TARGETS})
target_link_libraries(unit_test ${catkin_LIBRARIES})

