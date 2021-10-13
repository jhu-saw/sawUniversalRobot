#
# (C) Copyright 2014-2021 Johns Hopkins University (JHU), All Rights Reserved.
#
# --- begin cisst license - do not edit ---
#
# This software is provided "as is" under an open source license, with
# no warranty.  The complete license can be found in license.txt and
# http://www.cisst.org/cisst/license.txt.
#
# --- end cisst license ---

cmake_minimum_required (VERSION 3.0)
project (sawUniversalRobotExamples)

# Find cisst to define catkin macros so we can define the executable
# output path (it is enough to find cisstCommon)
find_package (cisst 1.1 COMPONENTS cisstCommon)

if (cisst_FOUND)

  # Load cisst configuration
  include (${CISST_USE_FILE})

  # catkin/ROS paths
  cisst_is_catkin_build (sawUniversalRobot_IS_CATKIN_BUILT)
  if (sawUniversalRobot_IS_CATKIN_BUILT)
    set (EXECUTABLE_OUTPUT_PATH "${CATKIN_DEVEL_PREFIX}/bin")
    set (LIBRARY_OUTPUT_PATH    "${CATKIN_DEVEL_PREFIX}/lib")
  else ()
    if (NOT LIBRARY_OUTPUT_PATH)
       set (LIBRARY_OUTPUT_PATH "${CMAKE_CURRENT_BINARY_DIR}/lib")
    endif (NOT LIBRARY_OUTPUT_PATH)
    if (NOT EXECUTABLE_OUTPUT_PATH)
       set (EXECUTABLE_OUTPUT_PATH "${CMAKE_CURRENT_BINARY_DIR}/bin")
    endif (NOT EXECUTABLE_OUTPUT_PATH)
  endif ()

  add_subdirectory (ConsoleTest)

  # Copy python file to the bin directory
  add_custom_target (sawUniversalRobotExamples ALL
                     ${CMAKE_COMMAND} -E copy
                     ${CMAKE_CURRENT_SOURCE_DIR}/python/ur_robot.py
                     ${EXECUTABLE_OUTPUT_PATH}/${CMAKE_CFG_INTDIR}/ur_robot.py
                     COMMENT "Copying ur_robot.py to ${EXECUTABLE_OUTPUT_PATH}/${CMAKE_CFG_INTDIR}")
endif (cisst_FOUND)
