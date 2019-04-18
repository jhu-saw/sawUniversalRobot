/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Anton Deguet
  Created on: 2017-02-22

  (C) Copyright 2017-2019 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/

#include <cisstCommon/cmnPath.h>
#include <cisstCommon/cmnUnits.h>
#include <cisstCommon/cmnCommandLineOptions.h>
#include <cisstCommon/cmnQt.h>
#include <cisstMultiTask/mtsTaskManager.h>
#include <cisstMultiTask/mtsSystemQtWidget.h>
#include <cisstParameterTypes/prmStateRobotQtWidget.h>
#include <sawUniversalRobot/mtsUniversalRobotScriptRT.h>

#include <ros/ros.h>
#include <cisst_ros_bridge/mtsROSBridge.h>

#include <QApplication>
#include <QMainWindow>


int main(int argc, char * argv[])
{
    // log configuration
    cmnLogger::SetMask(CMN_LOG_ALLOW_ALL);
    cmnLogger::SetMaskFunction(CMN_LOG_ALLOW_ALL);
    cmnLogger::SetMaskDefaultLog(CMN_LOG_ALLOW_ALL);
    cmnLogger::SetMaskClassMatching("mtsUniversalRobot", CMN_LOG_ALLOW_ALL);
    cmnLogger::AddChannel(std::cerr, CMN_LOG_ALLOW_ERRORS_AND_WARNINGS);

    // remove ROS args
    ros::V_string argout;
    ros::removeROSArgs(argc, argv, argout);
    argc = argout.size();

    // parse options
    cmnCommandLineOptions options;
    std::string ipAddress;
    double rosPeriod = 10.0 * cmn_ms;

    options.AddOptionOneValue("i", "ip-address",
                              "IP address for the UR controller",
                              cmnCommandLineOptions::REQUIRED_OPTION, &ipAddress);
    options.AddOptionOneValue("p", "ros-period",
                              "period in seconds to read all tool positions (default 0.01, 10 ms, 100Hz).  There is no point to have a period higher than the tracker component",
                              cmnCommandLineOptions::OPTIONAL_OPTION, &rosPeriod);

    typedef std::list<std::string> managerConfigType;
    managerConfigType managerConfig;
    options.AddOptionMultipleValues("m", "component-manager",
                                    "JSON file to configure component manager",
                                    cmnCommandLineOptions::OPTIONAL_OPTION, &managerConfig);

    // check that all required options have been provided
    std::string errorMessage;
    if (!options.Parse(argc, argv, errorMessage)) {
        std::cerr << "Error: " << errorMessage << std::endl;
        options.PrintUsage(std::cerr);
        return -1;
    }
    std::string arguments;
    options.PrintParsedArguments(arguments);
    std::cout << "Options provided:" << std::endl << arguments << std::endl;

    // create the components
    mtsUniversalRobotScriptRT * device = new mtsUniversalRobotScriptRT("UR");
    device->Configure(ipAddress);

    // add the components to the component manager
    mtsManagerLocal * componentManager = mtsComponentManager::GetInstance();
    componentManager->AddComponent(device);

    // ROS bridge
    mtsROSBridge * rosBridge = new mtsROSBridge("URBridge", rosPeriod, true, false); // spin, don't catch sigint

    // create a Qt user interface
    QApplication application(argc, argv);
    cmnQt::QApplicationExitsOnCtrlC();

    // organize all widgets in a tab widget
    QTabWidget * tabWidget = new QTabWidget;

    // configure all components

    // ROS publisher
    rosBridge->AddPublisherFromCommandRead<prmPositionCartesianGet, geometry_msgs::PoseStamped>
        ("Component", "GetPositionCartesian",
         "position_cartesian_current");

    rosBridge->AddPublisherFromCommandRead<prmStateJoint, sensor_msgs::JointState>
        ("Component", "GetStateJoint",
         "joint_states");

    rosBridge->AddSubscriberToCommandVoid("Component", "SetRobotFreeDriveMode", "SetRobotFreeDriveMode");
    rosBridge->AddSubscriberToCommandVoid("Component", "SetRobotRunningMode", "SetRobotRunningMode");
    rosBridge->AddSubscriberToCommandWrite<prmVelocityJointSet, sensor_msgs::JointState>
            ("Component", "JointVelocityMove", "JointVelocityMove");
    rosBridge->AddSubscriberToCommandWrite<prmPositionJointSet, sensor_msgs::JointState>
            ("Component", "JointPositionMove", "JointPositionMove");
    rosBridge->AddSubscriberToCommandWrite<prmVelocityCartesianSet, geometry_msgs::TwistStamped>
            ("Component", "CartesianVelocityMove", "CartesianVelocityMove");

    rosBridge->AddSubscriberToCommandWrite<prmPositionCartesianSet, geometry_msgs::PoseStamped>
            ("Component", "CartesianPositionMove", "CartesianPositionMove");

    rosBridge->AddSubscriberToCommandWrite<vctFrm3, geometry_msgs::PoseStamped>
        ("Component", "SetToolFrame", "SetToolFrame");

    rosBridge->AddLogFromEventWrite("Component", "Error",
                                    mtsROSEventWriteLog::ROS_LOG_ERROR);
    rosBridge->AddLogFromEventWrite("Component", "Warning",
                                    mtsROSEventWriteLog::ROS_LOG_WARN);
    rosBridge->AddLogFromEventWrite("Component", "Status",
                                    mtsROSEventWriteLog::ROS_LOG_INFO);

    // add the bridge after all interfaces have been created
    componentManager->AddComponent(rosBridge);
    componentManager->Connect(rosBridge->GetName(), "Component",
                              device->GetName(), "control");

    // Qt Widgets
    prmStateRobotQtWidgetComponent * stateWidget
        = new prmStateRobotQtWidgetComponent("UR-State");
    stateWidget->SetPrismaticRevoluteFactors(1.0 / cmn_mm, cmn180_PI);
    stateWidget->Configure();
    componentManager->AddComponent(stateWidget);
    componentManager->Connect(stateWidget->GetName(), "Component",
                              device->GetName(), "control");
    tabWidget->addTab(stateWidget, "State");

    mtsSystemQtWidgetComponent * systemWidget
        = new mtsSystemQtWidgetComponent("UR-System");
    systemWidget->Configure();
    componentManager->AddComponent(systemWidget);
    componentManager->Connect(systemWidget->GetName(), "Component",
                              device->GetName(), "control");
    tabWidget->addTab(systemWidget, "System");

    // custom user component
    const managerConfigType::iterator end = managerConfig.end();
    for (managerConfigType::iterator iter = managerConfig.begin();
         iter != end;
         ++iter) {
        if (!iter->empty()) {
            if (!cmnPath::Exists(*iter)) {
                CMN_LOG_INIT_ERROR << "File " << *iter
                                   << " not found!" << std::endl;
            } else {
                if (!componentManager->ConfigureJSON(*iter)) {
                    CMN_LOG_INIT_ERROR << "Configure: failed to configure component-manager" << std::endl;
                    return -1;
                }
            }
        }
    }

    // create and start all components
    componentManager->CreateAllAndWait(5.0 * cmn_s);
    componentManager->StartAllAndWait(5.0 * cmn_s);

    // run Qt user interface
    tabWidget->show();
    application.exec();

    // kill all components and perform cleanup
    componentManager->KillAllAndWait(5.0 * cmn_s);
    componentManager->Cleanup();

    cmnLogger::Kill();

    return 0;
}
