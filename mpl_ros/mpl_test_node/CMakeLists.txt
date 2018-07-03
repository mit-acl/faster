cmake_minimum_required(VERSION 2.8.3)
project(mpl_test_node)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wno-deprecated-declarations")

find_package(catkin_simple REQUIRED)
find_package(cmake_modules REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(motion_primitive_library REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR} ${MOTION_PRIMITIVE_LIBRARY_INCLUDE_DIRS})

catkin_simple()

cs_add_executable(test_primitive_map src/test_primitive_map.cpp)
target_link_libraries(test_primitive_map ${MOTION_PRIMITIVE_LIBRARY_LIBRARIES})
cs_add_executable(test_primitive_replan src/test_primitive_replan.cpp)
target_link_libraries(test_primitive_replan ${MOTION_PRIMITIVE_LIBRARY_LIBRARIES})
cs_add_executable(test_primitive_cloud src/test_primitive_cloud.cpp)
target_link_libraries(test_primitive_cloud ${MOTION_PRIMITIVE_LIBRARY_LIBRARIES})

cs_install()

cs_export()
