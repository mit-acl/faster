cmake_minimum_required(VERSION 2.8.3)
project(acl_planning)

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