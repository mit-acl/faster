cmake_minimum_required(VERSION 2.8)
project(jps3d)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -Wno-deprecated-declarations")

IF(NOT CMAKE_BUILD_TYPE)
  SET(CMAKE_BUILD_TYPE RelWithDebInfo)
ENDIF()

set(BUILD_SHARED_LIBS ON)


find_package(catkin REQUIRED COMPONENTS 
  roscpp
  pcl_ros
)

FIND_PACKAGE(roscpp  REQUIRED)
FIND_PACKAGE(Eigen3 REQUIRED)
FIND_PACKAGE(PkgConfig REQUIRED)
PKG_CHECK_MODULES(YAMLCPP REQUIRED yaml-cpp)

include_directories(${catkin_INCLUDE_DIRS})

IF(YAMLCPP_FOUND)
  MESSAGE(STATUS "Found yaml-cpp.")
ELSE()
  MESSAGE(FATAL_ERROR "Could not find yaml-cpp.")
ENDIF()

INCLUDE_DIRECTORIES(include ${EIGEN3_INCLUDE_DIRS} ${PCL_LIBRARY_DIRS})

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_library(jps_lib src/jps_planner/graph_search.cpp
  src/jps_planner/jps_planner.cpp ${PCL_LIBRARIES})

#add_library(dmp_lib src/distance_map_planner/graph_search.cpp
#  src/distance_map_planner/distance_map_planner.cpp ${PCL_LIBRARIES}) 

add_executable(create_map test/create_map.cpp)
target_link_libraries(create_map ${YAMLCPP_LIBRARIES} ${PCL_LIBRARIES})

include(CTest)

#add_executable(test_planner_2d test/test_planner_2d.cpp)
#target_link_libraries(test_planner_2d jps_lib ${YAMLCPP_LIBRARIES} ${PCL_LIBRARIES} ${Boost_LIBRARIES})
#add_test(test_planner_2d test_planner_2d ${CMAKE_SOURCE_DIR}/data/corridor.yaml)

#add_executable(test_planner_3d test/test_planner_3d.cpp)
#target_link_libraries(test_planner_3d jps_lib ${YAMLCPP_LIBRARIES} ${PCL_LIBRARIES})
#add_test(test_planner_3d test_planner_3d ${CMAKE_SOURCE_DIR}/data/simple3d.yaml)

#add_executable(test_distance_map_planner_2d test/test_distance_map_planner_2d.cpp)
#target_link_libraries(test_distance_map_planner_2d jps_lib dmp_lib ${YAMLCPP_LIBRARIES} ${Boost_LIBRARIES} ${PCL_LIBRARIES})
#add_test(test_distance_map_planner_2d test_distance_map_planner_2d ${CMAKE_SOURCE_DIR}/data/corridor.yaml)


install(FILES "${PROJECT_NAME}Config.cmake" "${PROJECT_NAME}ConfigVersion.cmake"
  DESTINATION "share/${PROJECT_NAME}/cmake")

#install(TARGETS jps_lib dmp_lib
#  RUNTIME DESTINATION bin
#  LIBRARY DESTINATION lib
#  ARCHIVE DESTINATION lib)


install(TARGETS jps_lib
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib)

install(DIRECTORY include/jps_basis include/jps_planner include/jps_collision
  DESTINATION include)