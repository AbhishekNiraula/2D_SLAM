# Install script for directory: /home/abhiniraula/Documents/Electronics Engineering/Sixth Semester/Minor Project/Minor Project Code/SLAM_Bot_Prototype/esp32_bot_description

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/abhiniraula/Documents/Electronics Engineering/Sixth Semester/Minor Project/Minor Project Code/SLAM_Bot_Prototype/install/esp32_bot")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

# Set default install directory permissions.
if(NOT DEFINED CMAKE_OBJDUMP)
  set(CMAKE_OBJDUMP "/usr/bin/objdump")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  include("/home/abhiniraula/Documents/Electronics Engineering/Sixth Semester/Minor Project/Minor Project Code/SLAM_Bot_Prototype/build/esp32_bot/ament_cmake_symlink_install/ament_cmake_symlink_install.cmake")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/esp32_bot" TYPE PROGRAM RENAME "tf_relay" FILES "/home/abhiniraula/Documents/Electronics Engineering/Sixth Semester/Minor Project/Minor Project Code/SLAM_Bot_Prototype/esp32_bot_description/scripts/tf_relay.py")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/esp32_bot" TYPE PROGRAM RENAME "explorer" FILES "/home/abhiniraula/Documents/Electronics Engineering/Sixth Semester/Minor Project/Minor Project Code/SLAM_Bot_Prototype/esp32_bot_description/scripts/explorer.py")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/esp32_bot" TYPE PROGRAM RENAME "rotate_scan" FILES "/home/abhiniraula/Documents/Electronics Engineering/Sixth Semester/Minor Project/Minor Project Code/SLAM_Bot_Prototype/esp32_bot_description/scripts/rotate_scan.py")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/esp32_bot" TYPE PROGRAM RENAME "stepped_rotate" FILES "/home/abhiniraula/Documents/Electronics Engineering/Sixth Semester/Minor Project/Minor Project Code/SLAM_Bot_Prototype/esp32_bot_description/scripts/stepped_rotate.py")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/esp32_bot" TYPE PROGRAM RENAME "tof_mapper" FILES "/home/abhiniraula/Documents/Electronics Engineering/Sixth Semester/Minor Project/Minor Project Code/SLAM_Bot_Prototype/esp32_bot_description/scripts/tof_mapper.py")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/esp32_bot" TYPE PROGRAM RENAME "wall_follower" FILES "/home/abhiniraula/Documents/Electronics Engineering/Sixth Semester/Minor Project/Minor Project Code/SLAM_Bot_Prototype/esp32_bot_description/scripts/wall_follower.py")
endif()

if(CMAKE_INSTALL_COMPONENT)
  set(CMAKE_INSTALL_MANIFEST "install_manifest_${CMAKE_INSTALL_COMPONENT}.txt")
else()
  set(CMAKE_INSTALL_MANIFEST "install_manifest.txt")
endif()

string(REPLACE ";" "\n" CMAKE_INSTALL_MANIFEST_CONTENT
       "${CMAKE_INSTALL_MANIFEST_FILES}")
file(WRITE "/home/abhiniraula/Documents/Electronics Engineering/Sixth Semester/Minor Project/Minor Project Code/SLAM_Bot_Prototype/build/esp32_bot/${CMAKE_INSTALL_MANIFEST}"
     "${CMAKE_INSTALL_MANIFEST_CONTENT}")
