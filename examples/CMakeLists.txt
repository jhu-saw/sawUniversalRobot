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

add_subdirectory(ConsoleTest)

# Copy python file to the bin directory
add_custom_target(sawUniversalRobot ALL
                  ${CMAKE_COMMAND} -E copy
                  ${CMAKE_CURRENT_SOURCE_DIR}/python/ur_robot.py
                  ${EXECUTABLE_OUTPUT_PATH}/${CMAKE_CFG_INTDIR}/ur_robot.py
                  COMMENT "Copying ur_robot.py to ${EXECUTABLE_OUTPUT_PATH}/${CMAKE_CFG_INTDIR}")
