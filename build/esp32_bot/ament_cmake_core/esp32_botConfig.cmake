# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_esp32_bot_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED esp32_bot_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(esp32_bot_FOUND FALSE)
  elseif(NOT esp32_bot_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(esp32_bot_FOUND FALSE)
  endif()
  return()
endif()
set(_esp32_bot_CONFIG_INCLUDED TRUE)

# output package information
if(NOT esp32_bot_FIND_QUIETLY)
  message(STATUS "Found esp32_bot: 0.0.0 (${esp32_bot_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'esp32_bot' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT esp32_bot_DEPRECATED_QUIET)
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(esp32_bot_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${esp32_bot_DIR}/${_extra}")
endforeach()
