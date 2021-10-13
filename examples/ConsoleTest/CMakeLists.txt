#
# (C) Copyright 2016-2021 Johns Hopkins University (JHU), All Rights Reserved.
#
# --- begin cisst license - do not edit ---
#
# This software is provided "as is" under an open source license, with
# no warranty.  The complete license can be found in license.txt and
# http://www.cisst.org/cisst/license.txt.
#
# --- end cisst license ---

cmake_minimum_required (VERSION 3.0)

# List cisst libraries needed
set (REQUIRED_CISST_LIBRARIES cisstCommon
                              cisstVector
                              cisstOSAbstraction
                              cisstMultiTask
                              cisstParameterTypes)

# find cisst and make sure the required libraries have been compiled
find_package (cisst 1.1 COMPONENTS ${REQUIRED_CISST_LIBRARIES})

if (cisst_FOUND_AS_REQUIRED)

  # load cisst configuration
  include (${CISST_USE_FILE})

  find_package (sawUniversalRobot 1.1.0 REQUIRED
                HINTS "${CMAKE_BINARY_DIR}/components")
  include_directories (${sawUniversalRobot_INCLUDE_DIR})
  link_directories (${sawUniversalRobot_LIBRARY_DIR})

  add_executable (sawUniversalRobotConsole main.cpp)

  # link with the cisst libraries
  cisst_target_link_libraries (sawUniversalRobotConsole ${REQUIRED_CISST_LIBRARIES})

  # link with sawUniversalRobot library
  target_link_libraries (sawUniversalRobotConsole ${sawUniversalRobot_LIBRARIES})

else (cisst_FOUND_AS_REQUIRED)
  message ("Information: sawUniversalRobotConsole will not be compiled, it requires ${REQUIRED_CISST_LIBRARIES}")
endif (cisst_FOUND_AS_REQUIRED)
