# - Config file for the decomp_util package
# It defines the following variables
#  DECOMP_UTIL_INCLUDE_DIRS - include directories for decomp_util
#  DECOMP_UTIL_LIBRARIES    - libraries to link against

# Compute paths
get_filename_component(DECOMP_UTIL_CMAKE_DIR "${CMAKE_CURRENT_LIST_FILE}" PATH)
set(DECOMP_UTIL_INCLUDE_DIRS "${DECOMP_UTIL_CMAKE_DIR}/../../../include")
