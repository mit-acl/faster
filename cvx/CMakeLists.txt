cmake_minimum_required(VERSION 2.8.3)
project(cvx)

## Add support for C++11, supported in ROS Kinetic and newer
add_definitions(-std=c++11)
#add_definitions(-std=c99)

find_package( Eigen3 REQUIRED )
include_directories(${EIGEN3_INCLUDE_DIR} ${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

#In the ws/ directory there must be the file FindGUROBI.cmake (available https://github.com/joschu/trajopt/blob/master/cmake/modules/FindGUROBI.cmake ). I had to modify it a little bit. 
list(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}")
find_package(GUROBI REQUIRED)

if(GUROBI_FOUND)
  message(STATUS "GUROBI FOUND")
else(GUROBI_FOUND)
  message(FATAL_ERROR "!!! GUROBI NOT FOUND !!!")
endif(GUROBI_FOUND)



include_directories(${GUROBI_INCLUDE_DIRS})
#set(GUROBI_LIBRARIES "${GUROBI_CXX_LIBRARY};${GUROBI_LIBRARY}" )


find_package(catkin REQUIRED COMPONENTS 
	roscpp
	rospy
	acl_msgs
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

set(GUROBI_LIBRARIES "$ENV{GUROBI_HOME}/lib/libgurobi_c++.a;$ENV{GUROBI_HOME}/lib/libgurobi80.so;$ENV{GUROBI_HOME}/lib/" )

add_executable(${PROJECT_NAME}_node src/main.cpp src/cvx.cpp src/solvers/solverGurobi.cpp)
target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${JPS3D_LIBRARIES} ${DECOMP_UTIL_LIBRARIES} ${CMAKE_CURRENT_LIST_DIR}/src/solvers/cvxgen/libcvx_vel.so ${CMAKE_CURRENT_LIST_DIR}/src/solvers/cvxgen/libcvx_accel.so ${CMAKE_CURRENT_LIST_DIR}/src/solvers/cvxgen/libcvx_jerk.so ${GUROBI_LIBRARIES})
add_dependencies(${PROJECT_NAME}_node ${catkin_EXPORTED_TARGETS} )


add_executable(gurobi_continuous_exec gurobi_continuous.cpp)
target_link_libraries(gurobi_continuous_exec ${GUROBI_LIBRARIES})

add_executable(gurobi_discrete_exec gurobi_discrete.cpp)

target_link_libraries(gurobi_discrete_exec ${GUROBI_LIBRARIES})

#add_executable(solverGurobi_exec src/solvers/solverGurobi.cpp)

#target_link_libraries(solverGurobi_exec ${GUROBI_LIBRARIES})



#add_executable(${PROJECT_NAME}_deubg_node src/debug.cpp)
#target_link_libraries(${PROJECT_NAME}_deubg_node ${catkin_LIBRARIES} ${CMAKE_CURRENT_LIST_DIR}/src/libcvx.so)
#add_dependencies(${PROJECT_NAME}_deubg_node ${catkin_EXPORTED_TARGETS})


