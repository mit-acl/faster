# - Config file for the motion_primitive_library package
# It defines the following variables
#  MOTION_PRIMITIVE_LIBRARY_INCLUDE_DIRS - include directories for motion_primitive_library
#  MOTION_PRIMITIVE_LIBRARY_LIBRARIES    - libraries to link against
 
# Compute paths
get_filename_component(MOTION_PRIMITIVE_LIBRARY_CMAKE_DIR "${CMAKE_CURRENT_LIST_FILE}" PATH)
set(MOTION_PRIMITIVE_LIBRARY_INCLUDE_DIRS "${MOTION_PRIMITIVE_LIBRARY_CMAKE_DIR}/../../../include")
 
set(MOTION_PRIMITIVE_LIBRARY_LIBRARIES "${MOTION_PRIMITIVE_LIBRARY_CMAKE_DIR}/../../../lib/libprimitive.so" 
  "${MOTION_PRIMITIVE_LIBRARY_CMAKE_DIR}/../../../lib/libpoly_solver.so"
  "${MOTION_PRIMITIVE_LIBRARY_CMAKE_DIR}/../../../lib/libmp_base_util.so"
  "${MOTION_PRIMITIVE_LIBRARY_CMAKE_DIR}/../../../lib/libmp_map_util.so"
  "${MOTION_PRIMITIVE_LIBRARY_CMAKE_DIR}/../../../lib/libmp_cloud_util.so")
