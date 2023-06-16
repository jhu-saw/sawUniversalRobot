#
# (C) Copyright 2014-2023 Johns Hopkins University (JHU), All Rights Reserved.
#
# --- begin cisst license - do not edit ---
#
# This software is provided "as is" under an open source license, with
# no warranty.  The complete license can be found in license.txt and
# http://www.cisst.org/cisst/license.txt.
#
# --- end cisst license ---

cmake_minimum_required (VERSION 3.10)
project (sawUniversalRobot VERSION 1.1.0)

set (REQUIRED_CISST_LIBRARIES
  cisstCommon
  cisstVector
  cisstOSAbstraction
  cisstMultiTask
  cisstParameterTypes)

find_package (cisst 1.1 REQUIRED ${REQUIRED_CISST_LIBRARIES})

if (cisst_FOUND_AS_REQUIRED)

  # Load cisst configuration
  include (${CISST_USE_FILE})

  # catkin/ROS paths
  cisst_set_output_path ()

  # create/configure file for find_package (sawUniversalRobot)
  set (sawUniversalRobot_INCLUDE_DIR
    "${CMAKE_CURRENT_SOURCE_DIR}/include"
    "${CMAKE_CURRENT_BINARY_DIR}/include")
  set (sawUniversalRobot_HEADER_DIR "${sawUniversalRobot_SOURCE_DIR}/include/sawUniversalRobot")
  set (sawUniversalRobot_LIBRARY_DIR "${LIBRARY_OUTPUT_PATH}")
  set (sawUniversalRobot_LIBRARIES sawUniversalRobot)

  # Allow c++ code to find local header files
  include_directories (${sawUniversalRobot_INCLUDE_DIR})

  # add all config files for this component
  cisst_add_config_files (sawUniversalRobot)

  add_library (sawUniversalRobot ${IS_SHARED}
    ${sawUniversalRobot_HEADER_DIR}/sawUniversalRobotExport.h
    ${sawUniversalRobot_HEADER_DIR}/mtsUniversalRobotScriptRT.h
    code/mtsUniversalRobotScriptRT.cpp)

  set_target_properties (sawUniversalRobot PROPERTIES
    VERSION ${sawUniversalRobot_VERSION}
    FOLDER "sawUniversalRobot")

  cisst_target_link_libraries (sawUniversalRobot
    ${REQUIRED_CISST_LIBRARIES})

  # Install target for headers and library
  install (DIRECTORY
    "${sawUniversalRobot_SOURCE_DIR}/include/sawUniversalRobot"
    "${sawUniversalRobot_BINARY_DIR}/include/sawUniversalRobot"
    DESTINATION include
    COMPONENT sawUniversalRobot-dev
    PATTERN .svn EXCLUDE)

  install (TARGETS sawUniversalRobot COMPONENT sawUniversalRobot
           RUNTIME DESTINATION bin
           LIBRARY DESTINATION lib
           ARCHIVE DESTINATION lib)

else (cisst_FOUND_AS_REQUIRED)

  message ("Information: code in ${CMAKE_CURRENT_SOURCE_DIR} will not be compiled, it requires ${REQUIRED_CISST_LIBRARIES}")

endif(cisst_FOUND_AS_REQUIRED)
