#
# (C) Copyright 2014-2019 Johns Hopkins University (JHU), All Rights Reserved.
#
# --- begin cisst license - do not edit ---
#
# This software is provided "as is" under an open source license, with
# no warranty.  The complete license can be found in license.txt and
# http://www.cisst.org/cisst/license.txt.
#
# --- end cisst license ---

project (sawUniversalRobot)

cmake_minimum_required (VERSION 2.8)

set (REQUIRED_CISST_LIBRARIES
     cisstCommon
     cisstVector
     cisstOSAbstraction
     cisstMultiTask
     cisstParameterTypes)

find_package (cisst 1.0.11 REQUIRED ${REQUIRED_CISST_LIBRARIES})

if (cisst_FOUND_AS_REQUIRED)

  # Load cisst configuration
  include (${CISST_USE_FILE})

  # catkin/ROS paths
  cisst_is_catkin_build (sawUniversalRobot_IS_CATKIN_BUILT)
  if (sawUniversalRobot_IS_CATKIN_BUILT)
    set (sawUniversalRobot_CONFIG_FILE_DIR
         "${CATKIN_DEVEL_PREFIX}/share/sawUniversalRobot/cmake")
    set (EXECUTABLE_OUTPUT_PATH "${CATKIN_DEVEL_PREFIX}/bin")
    set (LIBRARY_OUTPUT_PATH    "${CATKIN_DEVEL_PREFIX}/lib")
  else ()
    set (sawUniversalRobot_CONFIG_FILE_DIR "${sawUniversalRobot_BINARY_DIR}")
  endif ()

  set (CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

  # create/configure file for find_package (sawUniversalRobot)
  set (sawUniversalRobot_INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/include"
                                     "${CMAKE_CURRENT_BINARY_DIR}/include")
  set (sawUniversalRobot_LIBRARY_DIR "${LIBRARY_OUTPUT_PATH}")
  set (sawUniversalRobot_LIBRARIES sawUniversalRobot)

  # Set the version number
  set (sawUniversalRobot_VERSION_MAJOR "1")
  set (sawUniversalRobot_VERSION_MINOR "1")
  set (sawUniversalRobot_VERSION_PATCH "0")
  set (sawUniversalRobot_VERSION "${sawUniversalRobot_VERSION_MAJOR}.${sawUniversalRobot_VERSION_MINOR}.${sawUniversalRobot_VERSION_PATCH}")

  # Generate sawUniversalRobotRevision.h
  configure_file ("${CMAKE_CURRENT_SOURCE_DIR}/code/sawUniversalRobotRevision.h.in"
                  "${CMAKE_CURRENT_BINARY_DIR}/include/sawUniversalRobot/sawUniversalRobotRevision.h")

  include_directories (${sawUniversalRobot_INCLUDE_DIR})

  add_library (sawUniversalRobot ${IS_SHARED}
               include/sawUniversalRobot/sawUniversalRobotExport.h
               include/sawUniversalRobot/mtsUniversalRobotScriptRT.h
               code/mtsUniversalRobotScriptRT.cpp)
  set_property (TARGET sawUniversalRobot PROPERTY FOLDER "sawUniversalRobot")

  # Link with cisst libraries
  cisst_target_link_libraries (sawUniversalRobot
                               ${REQUIRED_CISST_LIBRARIES})

  set (sawUniversalRobot_CMAKE_CONFIG_FILE
       "${sawUniversalRobot_CONFIG_FILE_DIR}/sawUniversalRobotConfig.cmake")

  configure_file ("${CMAKE_CURRENT_SOURCE_DIR}/cmake/sawUniversalRobotConfig.cmake.in"
                  "${sawUniversalRobot_CMAKE_CONFIG_FILE}"
                  @ONLY)

  cisst_add_config_version ("${sawUniversalRobot_CONFIG_FILE_DIR}/sawUniversalRobotConfigVersion.cmake"
                            VERSION ${sawUniversalRobot_VERSION}
                            DESTINATION ${CISST_CMAKE_INSTALL_SUFFIX}/saw
                            COMPONENT sawUniversalRobot)

  # Install target for headers and library
  install (DIRECTORY
           "${sawUniversalRobot_SOURCE_DIR}/include/sawUniversalRobot"
           DESTINATION include
           PATTERN .svn EXCLUDE)

  install (TARGETS sawUniversalRobot
           RUNTIME DESTINATION bin
           LIBRARY DESTINATION lib
           ARCHIVE DESTINATION lib)

  install (FILES "${sawUniversalRobot_BINARY_DIR}/sawUniversalRobotConfig.cmake"
           DESTINATION "${CISST_CMAKE_INSTALL_SUFFIX}/saw")

else (cisst_FOUND_AS_REQUIRED)

  message ("Information: code in ${CMAKE_CURRENT_SOURCE_DIR} will not be compiled, it requires ${REQUIRED_CISST_LIBRARIES}")

endif(cisst_FOUND_AS_REQUIRED)
