#
# (C) Copyright 2014-2017 Johns Hopkins University (JHU), All Rights Reserved.
#
# --- begin cisst license - do not edit ---
#
# This software is provided "as is" under an open source license, with
# no warranty.  The complete license can be found in license.txt and
# http://www.cisst.org/cisst/license.txt.
#
# --- end cisst license ---

project(sawUniversalRobotController)

set (REQUIRED_CISST_LIBRARIES
     cisstCommon
     cisstVector
     cisstOSAbstraction
     cisstMultiTask
     cisstParameterTypes)

find_package (cisst REQUIRED ${REQUIRED_CISST_LIBRARIES})

if (cisst_FOUND_AS_REQUIRED)

  # Load cisst configuration
  include (${CISST_USE_FILE})

  set (CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

  if(CISST_BUILD_SHARED_LIBS)
    set(IS_SHARED SHARED)
  else(CISST_BUILD_SHARED_LIBS)
    set(IS_SHARED STATIC)
  endif(CISST_BUILD_SHARED_LIBS)

  # create/configure file for find_package (sawUniversalRobot)
  set (sawUniversalRobot_INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/include"
                                     "${CMAKE_CURRENT_BINARY_DIR}/include")
  set (sawUniversalRobot_LIBRARY_DIR "${LIBRARY_OUTPUT_PATH}")
  set (sawUniversalRobot_LIBRARIES sawUniversalRobotController)

  # Set the version number
  set (sawUniversalRobot_VERSION_MAJOR "1")
  set (sawUniversalRobot_VERSION_MINOR "0")
  set (sawUniversalRobot_VERSION_PATCH "0")
  set (sawUniversalRobot_VERSION "${sawUniversalRobot_VERSION_MAJOR}.${sawUniversalRobot_VERSION_MINOR}.${sawUniversalRobot_VERSION_PATCH}")

  # Generate sawUniversalRobotRevision.h
  configure_file ("${CMAKE_CURRENT_SOURCE_DIR}/code/sawUniversalRobotRevision.h.in"
                  "${CMAKE_CURRENT_BINARY_DIR}/include/sawUniversalRobot/sawUniversalRobotRevision.h")

  include_directories (${sawUniversalRobot_INCLUDE_DIR})

  add_library (sawUniversalRobotController ${IS_SHARED}
               include/sawUniversalRobot/mtsUniversalRobotScriptRT.h
               code/mtsUniversalRobotScriptRT.cpp)

  # Link with cisst libraries
  cisst_target_link_libraries(sawUniversalRobotController
                              ${REQUIRED_CISST_LIBRARIES})

  configure_file ("${CMAKE_CURRENT_SOURCE_DIR}/cmake/sawUniversalRobotConfig.cmake.in"
                  "${CMAKE_CURRENT_BINARY_DIR}/sawUniversalRobotConfig.cmake"
                  @ONLY)

else (cisst_FOUND_AS_REQUIRED)

  message ("Information: code in ${CMAKE_CURRENT_SOURCE_DIR} will not be compiled, it requires ${REQUIRED_CISST_LIBRARIES}")

endif(cisst_FOUND_AS_REQUIRED)
