#
# (C) Copyright 2016-2023 Johns Hopkins University (JHU), All Rights Reserved.
#
# --- begin cisst license - do not edit ---
#
# This software is provided "as is" under an open source license, with
# no warranty.  The complete license can be found in license.txt and
# http://www.cisst.org/cisst/license.txt.
#
# --- end cisst license ---

cmake_minimum_required (VERSION 3.10)
project (sawUniversalRobotConsoleQt VERSION 1.1.0)

# List cisst libraries needed
set (REQUIRED_CISST_LIBRARIES
  cisstCommon
  cisstCommonQt
  cisstVector
  cisstVectorQt
  cisstOSAbstraction
  cisstMultiTask
  cisstMultiTaskQt
  cisstParameterTypes
  cisstParameterTypesQt
  cisstQt)

# find cisst and make sure the required libraries have been compiled
find_package (cisst 1.2 COMPONENTS ${REQUIRED_CISST_LIBRARIES})

if (cisst_FOUND_AS_REQUIRED)

  # load cisst configuration
  include (${CISST_USE_FILE})

  # catkin/ROS paths
  cisst_set_output_path ()

  find_package (sawUniversalRobot 1.1.0 REQUIRED
                HINTS "${CMAKE_BINARY_DIR}/components")
  include_directories (${sawUniversalRobot_INCLUDE_DIR})
  link_directories (${sawUniversalRobot_LIBRARY_DIR})

  add_executable (sawUniversalRobotConsoleQt mainQt.cpp)
  set_target_properties (sawUniversalRobotConsoleQt PROPERTIES
    COMPONENT sawUniversalRobot-Examples
    FOLDER "sawNDITracker")


  # link with the cisst libraries
  cisst_target_link_libraries (sawUniversalRobotConsoleQt ${REQUIRED_CISST_LIBRARIES})

  # link with sawUniversalRobot library
  target_link_libraries (sawUniversalRobotConsoleQt ${sawUniversalRobot_LIBRARIES})

  install (TARGETS sawUniversalRobotConsoleQt COMPONENT sawUniversalRobot-Examples
    RUNTIME DESTINATION bin
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib)

else (cisst_FOUND_AS_REQUIRED)
  message ("Information: sawUniversalRobotConsoleQt will not be compiled, it requires ${REQUIRED_CISST_LIBRARIES}")
endif (cisst_FOUND_AS_REQUIRED)
