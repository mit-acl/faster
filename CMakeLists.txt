cmake_minimum_required(VERSION 2.8.3)
project(acl_planning)

# need to latest c++ compiler standard
add_compile_options(-std=c++11)

find_package( Eigen3 REQUIRED )
include_directories(${EIGEN3_INCLUDE_DIR})


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

add_executable(reactive_planner src/react/main.cpp src/react/rp.cpp )
add_dependencies(reactive_planner ${catkin_EXPORTED_TARGETS})
target_link_libraries(reactive_planner ${catkin_LIBRARIES})

#add_executable(unit_test src/react/test.cpp src/react/rp.cpp )
#add_dependencies(unit_test ${catkin_EXPORTED_TARGETS})
#target_link_libraries(unit_test ${catkin_LIBRARIES})

add_executable(gpf src/gpf/main.cpp src/gpf/gpf.cpp)
add_dependencies(gpf ${catkin_EXPORTED_TARGETS})
target_link_libraries(gpf ${catkin_LIBRARIES})
