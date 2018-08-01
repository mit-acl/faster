cmake_minimum_required(VERSION 2.8.3)
project(planning_ros_utils)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -Wno-deprecated-declarations")
find_package(catkin REQUIRED COMPONENTS rviz roscpp)
find_package(catkin_simple REQUIRED)
find_package(cmake_modules)
find_package(Eigen3 REQUIRED)
FIND_PACKAGE(VTK REQUIRED)
FIND_PACKAGE(motion_primitive_library REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIRS} ${MOTION_PRIMITIVE_LIBRARY_INCLUDE_DIRS})


add_definitions(-DQT_NO_KEYWORDS)

## This setting causes Qt's "MOC" generation to happen automatically.
## this does not moc things in include!!!!!!! only in src
set(CMAKE_AUTOMOC ON)

## We'll use the version that rviz used so they are compatible.
if(rviz_QT_VERSION VERSION_LESS "5")
  message(STATUS "Using Qt4 based on the rviz_QT_VERSION: ${rviz_QT_VERSION}")
  find_package(Qt4 ${rviz_QT_VERSION} EXACT REQUIRED QtCore QtGui)
  include(${QT_USE_FILE})
else()
  message(STATUS "Using Qt5 based on the rviz_QT_VERSION: ${rviz_QT_VERSION}")
  find_package(Qt5 ${rviz_QT_VERSION} EXACT REQUIRED Core Widgets)
  set(QT_LIBRARIES Qt5::Widgets)
endif()

catkin_simple()

if(NOT VTK_FOUND)
  set(DEFAULT FALSE)
  set(REASON "VTK was not found.")
  message("VTK was not found")
else(NOT VTK_FOUND)
  include(${VTK_USE_FILE})
  cs_add_executable(mesh_sampling src/planning_ros_utils/mesh_sampling.cpp)
endif(NOT VTK_FOUND)

cs_add_library(voxel_grid src/planning_ros_utils/voxel_grid.cpp)

set(SOURCE_FILES
  src/planning_rviz_plugins/bound_visual.cpp
  src/planning_rviz_plugins/mesh_visual.cpp
  src/planning_rviz_plugins/primitive_visual.cpp
  src/planning_rviz_plugins/trajectory_visual.cpp
  src/planning_rviz_plugins/path_visual.cpp
  src/planning_rviz_plugins/map_display.cpp
  src/planning_rviz_plugins/path_display.cpp
  src/planning_rviz_plugins/path_array_display.cpp
  src/planning_rviz_plugins/primitives_display.cpp
  src/planning_rviz_plugins/trajectory_display.cpp
  src/planning_ros_utils/primitive_ros_utils.cpp
  ${MOC_FILES})

cs_add_library(planning_rviz_plugins ${SOURCE_FILES})
target_link_libraries(planning_rviz_plugins ${QT_LIBRARIES} ${catkin_LIBRARIES} ${rviz_DEFAULT_PLUGIN_LIBRARIES} ${MOTION_PRIMITIVE_LIBRARY_LIBRARIES})

cs_install()

cs_export()
