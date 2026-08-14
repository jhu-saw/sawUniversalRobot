#
# (C) Copyright 2016-2026 Johns Hopkins University (JHU), All Rights Reserved.
#
# --- begin cisst license - do not edit ---
#
# This software is provided "as is" under an open source license, with
# no warranty.  The complete license can be found in license.txt and
# http://www.cisst.org/cisst/license.txt.
#
# --- end cisst license ---

cmake_minimum_required (VERSION 3.16)
project (sawUniversalRobotConsoleTest VERSION 1.2.0)

# List cisst libraries needed
set (REQUIRED_CISST_LIBRARIES cisstCommon
                              cisstVector
                              cisstOSAbstraction
                              cisstMultiTask
                              cisstParameterTypes)

# find cisst and make sure the required libraries have been compiled
find_package (cisst 1.5 REQUIRED COMPONENTS ${REQUIRED_CISST_LIBRARIES})

find_package (sawUniversalRobot 1.1.0 REQUIRED
              HINTS "${CMAKE_BINARY_DIR}/components")
include_directories (${sawUniversalRobot_INCLUDE_DIR})
link_directories (${sawUniversalRobot_LIBRARY_DIR})

add_executable (sawUniversalRobotConsole main.cpp)

target_link_libraries (sawUniversalRobotConsole
                       ${sawUniversalRobot_LIBRARIES}
                       ${cisst_LIBRARIES})

set_target_properties (sawUniversalRobotConsole PROPERTIES
  COMPONENT sawUniversalRobot-Examples
  FOLDER "sawUniversalRobot")

install (TARGETS sawUniversalRobotConsole COMPONENT sawUniversalRobot-Examples
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib)
