cmake_minimum_required(VERSION 2.8.3)
project(faster)

## Add support for C++11, supported in ROS Kinetic and newer
add_definitions(-std=c++11)
#add_definitions(-std=c99)

find_package( Eigen3 REQUIRED )
include_directories(${EIGEN3_INCLUDE_DIR} ${PCL_INCLUDE_DIRS} include)
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

list(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}")
find_package(GUROBI REQUIRED)

if(GUROBI_FOUND)
  message(STATUS "GUROBI FOUND")
else(GUROBI_FOUND)
  message(FATAL_ERROR "GUROBI NOT FOUND!")
endif(GUROBI_FOUND)


set(CMAKE_BUILD_TYPE Release) #From terminal: catkin config -DCMAKE_BUILD_TYPE=Release (or RelWithDebInfo)

include_directories(${GUROBI_INCLUDE_DIRS})
#set(GUROBI_LIBRARIES "${GUROBI_CXX_LIBRARY};${GUROBI_LIBRARY}" )


find_package(catkin REQUIRED COMPONENTS 
	roscpp
	rospy
	snapstack_msgs
	faster_msgs
	sensor_msgs
	pcl_ros
	decomp_util
	decomp_ros_utils
)
find_package(decomp_util REQUIRED)
include_directories(${DECOMP_UTIL_INCLUDE_DIRS})

find_package(jps3d REQUIRED)
include_directories(${JPS3D_INCLUDE_DIRS})

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES cvx
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

include_directories(${catkin_INCLUDE_DIRS})

FILE(GLOB GurobiSOFiles $ENV{GUROBI_HOME}/lib/libgurobi*[0-9].so) #files that are start with libgurobi and end with number.so
set(GUROBI_LIBRARIES "$ENV{GUROBI_HOME}/lib/libgurobi_c++.a;${GurobiSOFiles};$ENV{GUROBI_HOME}/lib/" )

add_executable(${PROJECT_NAME}_node src/main.cpp src/faster.cpp src/faster_ros.cpp src/utils.cpp  src/jps_manager.cpp src/solverGurobi.cpp)
target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${JPS3D_LIBRARIES} ${DECOMP_UTIL_LIBRARIES} ${GUROBI_LIBRARIES})
add_dependencies(${PROJECT_NAME}_node ${catkin_EXPORTED_TARGETS} )


# add_executable(gurobi_continuous_exec gurobi_continuous.cpp)
# target_link_libraries(gurobi_continuous_exec ${GUROBI_LIBRARIES})
# add_executable(gurobi_discrete_exec gurobi_discrete.cpp)
# target_link_libraries(gurobi_discrete_exec ${GUROBI_LIBRARIES})



