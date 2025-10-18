# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_rs_to_velodyne_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED rs_to_velodyne_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(rs_to_velodyne_FOUND FALSE)
  elseif(NOT rs_to_velodyne_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(rs_to_velodyne_FOUND FALSE)
  endif()
  return()
endif()
set(_rs_to_velodyne_CONFIG_INCLUDED TRUE)

# output package information
if(NOT rs_to_velodyne_FIND_QUIETLY)
  message(STATUS "Found rs_to_velodyne: 0.0.0 (${rs_to_velodyne_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'rs_to_velodyne' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${rs_to_velodyne_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(rs_to_velodyne_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${rs_to_velodyne_DIR}/${_extra}")
endforeach()
