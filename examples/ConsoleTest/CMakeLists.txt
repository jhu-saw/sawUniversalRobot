#
# (C) Copyright 2016-2017 Johns Hopkins University (JHU), All Rights Reserved.
#
# --- begin cisst license - do not edit ---
#
# This software is provided "as is" under an open source license, with
# no warranty.  The complete license can be found in license.txt and
# http://www.cisst.org/cisst/license.txt.
#
# --- end cisst license ---

cmake_minimum_required (VERSION 2.8)

# List cisst libraries needed
set (REQUIRED_CISST_LIBRARIES cisstCommon
                              cisstVector
                              cisstOSAbstraction
                              cisstMultiTask
                              cisstParameterTypes)

# find cisst and make sure the required libraries have been compiled
find_package (cisst 1.0.8 COMPONENTS ${REQUIRED_CISST_LIBRARIES})

if (cisst_FOUND_AS_REQUIRED)

  # load cisst configuration
  include (${CISST_USE_FILE})

  find_package (sawUniversalRobot REQUIRED)
  include_directories (${sawUniversalRobot_INCLUDE_DIR})
  link_directories (${sawUniversalRobot_LIBRARY_DIR})

  add_executable (URconsole main.cpp)

  # link with the cisst libraries
  cisst_target_link_libraries (URconsole ${REQUIRED_CISST_LIBRARIES})

  # link with sawUniversalRobot library
  target_link_libraries (URconsole ${sawUniversalRobot_LIBRARIES})

else (cisst_FOUND_AS_REQUIRED)
  message ("Information: URconsole will not be compiled, it requires ${REQUIRED_CISST_LIBRARIES}")
endif (cisst_FOUND_AS_REQUIRED)
