# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_go1_tools_high_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED go1_tools_high_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(go1_tools_high_FOUND FALSE)
  elseif(NOT go1_tools_high_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(go1_tools_high_FOUND FALSE)
  endif()
  return()
endif()
set(_go1_tools_high_CONFIG_INCLUDED TRUE)

# output package information
if(NOT go1_tools_high_FIND_QUIETLY)
  message(STATUS "Found go1_tools_high: 0.0.0 (${go1_tools_high_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'go1_tools_high' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${go1_tools_high_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(go1_tools_high_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${go1_tools_high_DIR}/${_extra}")
endforeach()
