# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_mission_3_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED mission_3_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(mission_3_FOUND FALSE)
  elseif(NOT mission_3_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(mission_3_FOUND FALSE)
  endif()
  return()
endif()
set(_mission_3_CONFIG_INCLUDED TRUE)

# output package information
if(NOT mission_3_FIND_QUIETLY)
  message(STATUS "Found mission_3: 0.0.0 (${mission_3_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'mission_3' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${mission_3_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(mission_3_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${mission_3_DIR}/${_extra}")
endforeach()
