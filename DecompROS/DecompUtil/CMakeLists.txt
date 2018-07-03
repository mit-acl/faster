cmake_minimum_required(VERSION 2.8.3)
project(decomp_util)

# set default build type
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif()
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -Wno-deprecated-declarations")

set(BUILD_SHARED_LIBS ON)

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIRS} include)

add_library(geometry_utils src/geometry_utils.cpp)

add_library(seed_decomp src/seed_decomp.cpp
  src/geometry_utils.cpp)

add_library(ellipse_decomp
  src/geometry_utils.cpp
  src/line_segment.cpp
  src/ellipse_decomp.cpp)

add_library(iterative_decomp src/iterative_decomp.cpp)
target_link_libraries(iterative_decomp ellipse_decomp)

add_library(ellipsoid_utils src/ellipsoid_utils.cpp)
target_link_libraries(ellipsoid_utils geometry_utils)

install(FILES "${PROJECT_NAME}Config.cmake" "${PROJECT_NAME}ConfigVersion.cmake"
  DESTINATION "share/${PROJECT_NAME}/cmake")

install(TARGETS geometry_utils seed_decomp ellipse_decomp iterative_decomp ellipsoid_utils 
  RUNTIME DESTINATION bin                                                                               
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME})
