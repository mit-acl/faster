cmake_minimum_required(VERSION 2.8.3)
project(cvx)

## Add support for C++11, supported in ROS Kinetic and newer
add_definitions(-std=c++11)
#add_definitions(-std=c99)

find_package( Eigen3 REQUIRED )
include_directories(${EIGEN3_INCLUDE_DIR} ${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

find_package(catkin REQUIRED COMPONENTS 
	roscpp
	rospy
	acl_msgs
	sensor_msgs
	pcl_ros
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES cvx
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

include_directories(${catkin_INCLUDE_DIRS})

add_executable(${PROJECT_NAME}_node src/main.cpp src/cvx.cpp)
target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES} ${PCL_LIBRARIES}  ${CMAKE_CURRENT_LIST_DIR}/src/libcvx.so)
add_dependencies(${PROJECT_NAME}_node ${catkin_EXPORTED_TARGETS})

#add_executable(${PROJECT_NAME}_deubg_node src/debug.cpp)
#target_link_libraries(${PROJECT_NAME}_deubg_node ${catkin_LIBRARIES} ${CMAKE_CURRENT_LIST_DIR}/src/libcvx.so)
#add_dependencies(${PROJECT_NAME}_deubg_node ${catkin_EXPORTED_TARGETS})


