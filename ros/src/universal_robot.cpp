/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Anton Deguet, Peter Kazanzides
  Created on: 2017-02-22

  (C) Copyright 2017-2023 Johns Hopkins University (JHU), All Rights Reserved.

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
#include <cisst_ros_crtk/mts_ros_crtk_bridge_provided.h>

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

    // create ROS node handle
    ros::init(argc, argv, "universal_robot", ros::init_options::AnonymousName);
    ros::NodeHandle rosNodeHandle;

    // parse options
    cmnCommandLineOptions options;
    std::string ipAddress;
    double rosPeriod = 10.0 * cmn_ms;
    double tfPeriod = 20.0 * cmn_ms;

    options.AddOptionOneValue("i", "ip-address",
                              "IP address for the UR controller",
                              cmnCommandLineOptions::REQUIRED_OPTION, &ipAddress);
    options.AddOptionNoValue("D", "dark-mode",
                             "replaces the default Qt palette with darker colors");
    options.AddOptionOneValue("p", "ros-period",
                              "period in seconds to read all tool positions (default 0.01, 10 ms, 100Hz).  There is no point to have a period higher than the UR component",
                              cmnCommandLineOptions::OPTIONAL_OPTION, &rosPeriod);
    options.AddOptionOneValue("P", "tf-ros-period",
                              "period in seconds to read all components and broadcast tf2 (default 0.02, 20 ms, 50Hz).  There is no point to have a period higher than the UR's period",
                              cmnCommandLineOptions::OPTIONAL_OPTION, &tfPeriod);

    std::list<std::string> managerConfig;
    options.AddOptionMultipleValues("m", "component-manager",
                                    "JSON file to configure component manager",
                                    cmnCommandLineOptions::OPTIONAL_OPTION, &managerConfig);

    // check that all required options have been provided
    if (!options.Parse(argc, argv, std::cerr)) {
        return -1;
    }
    std::string arguments;
    options.PrintParsedArguments(arguments);
    std::cout << "Options provided:" << std::endl << arguments << std::endl;

    // create the components
    mtsUniversalRobotScriptRT * ur = new mtsUniversalRobotScriptRT("UR");
    ur->Configure(ipAddress);

    // add the components to the component manager
    mtsManagerLocal * componentManager = mtsComponentManager::GetInstance();
    componentManager->AddComponent(ur);

    // create a Qt user interface
    QApplication application(argc, argv);
    cmnQt::QApplicationExitsOnCtrlC();
    if (options.IsSet("dark-mode")) {
        cmnQt::SetDarkMode();
    }

    // organize all widgets in a tab widget
    QTabWidget * tabWidget = new QTabWidget;

    // Qt Widgets
    prmStateRobotQtWidgetComponent * stateWidget
        = new prmStateRobotQtWidgetComponent("UR-State");
    stateWidget->SetPrismaticRevoluteFactors(1.0 / cmn_mm, cmn180_PI);
    stateWidget->Configure();
    componentManager->AddComponent(stateWidget);
    componentManager->Connect(stateWidget->GetName(), "Component",
                              ur->GetName(), "control");
    tabWidget->addTab(stateWidget, "State");

    mtsSystemQtWidgetComponent * systemWidget
        = new mtsSystemQtWidgetComponent("UR-System");
    systemWidget->Configure();
    componentManager->AddComponent(systemWidget);
    componentManager->Connect(systemWidget->GetName(), "Component",
                              ur->GetName(), "control");
    tabWidget->addTab(systemWidget, "System");

    // ROS CRTK bridge
    mts_ros_crtk_bridge_provided * crtk_bridge
        = new mts_ros_crtk_bridge_provided("ur_crtk_bridge", &rosNodeHandle);
    crtk_bridge->bridge_interface_provided(ur->GetName(),
                                           "control",
                                           "", // ros sub namespace
                                           rosPeriod, tfPeriod);

    // extra subscribers
    crtk_bridge->subscribers_bridge()
        .AddSubscriberToCommandVoid
        ("control", "SetRobotFreeDriveMode", "SetRobotFreeDriveMode");
    crtk_bridge->subscribers_bridge()
        .AddSubscriberToCommandVoid
        ("control", "SetRobotRunningMode", "SetRobotRunningMode");
    crtk_bridge->subscribers_bridge()
        .AddSubscriberToCommandWrite<vctFrm3, geometry_msgs::PoseStamped>
        ("control", "SetToolFrame", "SetToolFrame");

    componentManager->AddComponent(crtk_bridge);
    crtk_bridge->Connect();
    componentManager->Connect(crtk_bridge->subscribers_bridge().GetName(),
                              "control",
                              ur->GetName(),
                              "control");

    // custom user component
    if (!componentManager->ConfigureJSON(managerConfig)) {
        CMN_LOG_INIT_ERROR << "Configure: failed to configure component-manager, check cisstLog for error messages" << std::endl;
        return -1;
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
