
# sawUniversalRobot

This SAW package contains a component to interface to a Universal Robot (UR3/UR5/UR10/UR16/UR20) via
the real-time scripting/socket interface (mtsUniversalRobotScriptRT). It has been tested with multiple
versions of the Universal Robot controller, including CB2, CB3 and the e-Series.
For the e-Series, it is necessary to turn on Remote Control mode; otherwise, the robot will not
respond to any motion commands (or any other commands that change the state of the robot).

This component runs on an external PC and communicates with the UR controller via a TCP/IP socket
to port 30003. It also connects to the Dashboard Server via port 29999.

The `ros` folder contains code for a ROS node that interfaces with the
sawUniversalRobot component and publishes the 3D transformations as
well as the joint state (position, velocity and effort).  To build
the ROS node, make sure you use `catkin build`.


# Links
 * License: http://github.com/jhu-cisst/cisst/blob/master/license.txt
 * JHU-LCSR software: http://jhu-lcsr.github.io/software/

# Dependencies
 * cisst libraries: https://github.com/jhu-cisst/cisst
 * Qt for user interface
 * ROS (optional)

# Running the examples

## ROS

First you need to make sure you can communicate with the UR robot using it's IP address (for example 10.162.34.61).
The ROS node is `universal_robot` and can be found in the package `universal_robot_ros`:
```sh
rosrun universal_robot_ros universal_robot -i 10.162.34.61
```

## Console

The `sawUniversalRobotConsole` program requires the IP address to be provided as a command line argument.
Press the 'h' key to display a list of commands.