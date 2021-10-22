/*-*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-   */
/*ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab:*/

/*
(C) Copyright 2016-2021 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/

#include <cisstConfig.h>
#include <cisstCommon/cmnLogger.h>
#include <cisstCommon/cmnKbHit.h>
#include <cisstCommon/cmnGetChar.h>
#include <cisstCommon/cmnConstants.h>
#include <cisstVector/vctRodriguezRotation3.h>
#include <cisstVector/vctEulerRotation3.h>
#include <cisstOSAbstraction/osaSleep.h>
#include <cisstMultiTask/mtsManagerLocal.h>
#include <cisstMultiTask/mtsTaskContinuous.h>
#include <cisstMultiTask/mtsInterfaceRequired.h>
#include <cisstMultiTask/mtsManagerLocal.h>
#include <sawUniversalRobot/mtsUniversalRobotScriptRT.h>


class UniversalRobotClient : public mtsTaskMain {

private:
    prmStateJoint m_measured_js;
    prmPositionCartesianGet m_measured_cp;
    vctDoubleVec jtgoal, jtvel;
    prmPositionJointSet jtposSet;
    prmPositionCartesianSet cartposSet;
    prmVelocityJointSet jtvelSet;
    prmVelocityCartesianSet cartVelSet;
    std::string buffer;

    mtsFunctionRead GetControllerTime;
    mtsFunctionRead GetControllerExecTime;
    mtsFunctionRead measured_js;
    mtsFunctionRead measured_cp;
    mtsFunctionRead GetConnected;
    mtsFunctionRead GetVersion;
    mtsFunctionRead GetAveragePeriod;
    mtsFunctionWrite move_jp;
    mtsFunctionWrite move_cp;
    mtsFunctionWrite move_cr;
    mtsFunctionWrite servo_jv;
    mtsFunctionWrite servo_cp;
    mtsFunctionRead GetDebug;
    mtsFunctionWrite SetPayload;
    mtsFunctionWrite SetToolVoltage;
    mtsFunctionWrite ShowPopup;
    mtsFunctionWrite SendToDashboardServer;
    mtsFunctionRead GetRobotMode;
    mtsFunctionRead GetJointModes;
    mtsFunctionRead GetSafetyMode;
    mtsFunctionRead IsMotorPowerOn;
    mtsFunctionRead IsEStop;
    mtsFunctionVoid SetRobotFreeDriveMode;
    mtsFunctionVoid SetRobotRunningMode;
    mtsFunctionVoid StopMotion;
    mtsFunctionVoid EnableMotorPower;
    mtsFunctionVoid DisableMotorPower;

    bool debugMode;
    bool cartMode;

    void OnSocketError(void) {
        std::cout << std::endl << "Socket error communicating with robot" << std::endl;
    }
    void OnRobotNotReady(void) {
        std::cout << std::endl << "Robot not ready for motion command" << std::endl;
    }
    void OnReceiveTimeout(void) {
        std::cout << std::endl << "Timeout receiving data from robot" << std::endl;
    }
    void OnPacketInvalid(const vctULong2 &len) {
        std::cout << std::endl << "Invalid packet from robot, numBytes = " << len[0]
                  << ", packageLength = " << len[1] << std::endl;
    }

    void OnErrorEvent(const mtsMessage &msg) {
        std::cout << std::endl << "Error: " << msg.Message << std::endl;
    }
    void OnWarningEvent(const mtsMessage &msg) {
        std::cout << std::endl << "Warning: " << msg.Message << std::endl;
    }
    void OnStatusEvent(const mtsMessage &msg) {
        std::cout << std::endl << "Status: " << msg.Message << std::endl;
    }

    void GetCartesianPose(prmPositionCartesianSet &cartposSet);

public:

    UniversalRobotClient() : mtsTaskMain("UniversalRobotClient"),
                             jtgoal(6), jtvel(6), jtposSet(6), jtvelSet(6),
                             debugMode(false), cartMode(false)
    {
        m_measured_js.SetSize(6);
        mtsInterfaceRequired *req = AddInterfaceRequired("Input", MTS_OPTIONAL);
        if (req) {
            req->AddFunction("GetControllerTime", GetControllerTime);
            req->AddFunction("GetControllerExecTime", GetControllerExecTime);
            req->AddFunction("measured_js", measured_js);
            req->AddFunction("measured_cp", measured_cp);
            req->AddFunction("GetConnected", GetConnected);
            req->AddFunction("GetAveragePeriod", GetAveragePeriod);
            req->AddFunction("move_jp", move_jp);
            req->AddFunction("move_cp", move_cp);
            req->AddFunction("move_cr", move_cr);
            req->AddFunction("servo_jv", servo_jv);
            req->AddFunction("servo_cv", servo_cp);
            req->AddFunction("GetDebug", GetDebug);
            req->AddFunction("SetPayload", SetPayload);
            req->AddFunction("SetToolVoltage", SetToolVoltage);
            req->AddFunction("ShowPopup", ShowPopup);
            req->AddFunction("SendToDashboardServer", SendToDashboardServer);
            req->AddFunction("GetVersion", GetVersion);
            req->AddFunction("GetRobotMode", GetRobotMode, MTS_OPTIONAL);
            req->AddFunction("GetJointModes", GetJointModes, MTS_OPTIONAL);
            req->AddFunction("GetSafetyMode", GetSafetyMode, MTS_OPTIONAL);
            req->AddFunction("IsMotorPowerOn", IsMotorPowerOn);
            req->AddFunction("IsEStop", IsEStop);
            req->AddFunction("SetRobotFreeDriveMode", SetRobotFreeDriveMode);
            req->AddFunction("SetRobotRunningMode", SetRobotRunningMode);
            req->AddFunction("StopMotion", StopMotion);
            req->AddFunction("EnableMotorPower", EnableMotorPower);
            req->AddFunction("DisableMotorPower", DisableMotorPower);
            req->AddEventHandlerVoid(&UniversalRobotClient::OnSocketError,
                                     this, "SocketError");
            req->AddEventHandlerVoid(&UniversalRobotClient::OnRobotNotReady,
                                     this, "RobotNotReady");
            req->AddEventHandlerVoid(&UniversalRobotClient::OnReceiveTimeout,
                                     this, "ReceiveTimeout");
            req->AddEventHandlerWrite(&UniversalRobotClient::OnPacketInvalid,
                                     this, "PacketInvalid");
            req->AddEventHandlerWrite(&UniversalRobotClient::OnErrorEvent, this, "Error");
            req->AddEventHandlerWrite(&UniversalRobotClient::OnWarningEvent,this, "Warning");
            req->AddEventHandlerWrite(&UniversalRobotClient::OnStatusEvent, this, "Status");
        }
    }

    void Configure(const std::string&) {}

    void PrintHelp()
    {
        std::cout << "Available commands:" << std::endl
                  << "  h: display help information" << std::endl
                  << "  m: position move joints" << std::endl
                  << "  M: position move cartesian" << std::endl
                  << "  v: velocity move joints" << std::endl
                  << "  V: velocity move cartesian" << std::endl
                  << "  c: toggle Cartesian/joint pose display" << std::endl
                  << "  d: toggle debug data display" << std::endl
                  << "  D: send command to dashboard server" << std::endl
                  << "  p: set payload" << std::endl
                  << "  P: show popup message" << std::endl
                  << "  R: relative position move cartesian" << std::endl
                  << "  s: stop motion" << std::endl
                  << "  x: get version" << std::endl
                  << "  f: free drive mode" << std::endl
                  << "  r: running mode" << std::endl
                  << "  i: display robot info (e.g., times, modes)" << std::endl
                  << "  e: enable motor power" << std::endl
                  << "  n: disable motor power" << std::endl
                  << "  t: set tool voltage (0, 12, or 24)" << std::endl
                  << "  q: quit" << std::endl;
    }

    void Startup()
    {
        PrintHelp();
    }

    void Run() {

        bool connected = false;
        double period, cTime, cExecTime;
        vct3 velxyz, velrot;
        vct6 debug;
        double payload;
        int toolVoltage;
        int version;
        int robotMode;
        vctInt6 jointModes;
        int safetyMode;
        bool flag;
        const char *versionString[] = { "Unknown", "Pre-1.8", "1.8", "3.0-3.1", "3.2-3.4", "3.5-3.9", "3.10-3.11" };
        size_t i;

        ProcessQueuedEvents();

        GetDebug(debug);
        GetControllerTime(cTime);
        GetControllerExecTime(cExecTime);
        measured_js(m_measured_js);
        measured_cp(m_measured_cp);
        GetConnected(connected);
        GetAveragePeriod(period);

        if (connected) {
            cartposSet.SetMask(vctBool2(false, false));
            if (cmnKbHit()) {
                char c = cmnGetChar();
                switch (c) {

                case 'h':
                    std::cout << std::endl;
                    PrintHelp();
                    break;

                case 'm':   // position move joint
                    std::cout << std::endl << "Enter joint positions (deg): ";
                    std::cin >> jtgoal[0] >> jtgoal[1] >> jtgoal[2] >> jtgoal[3] >> jtgoal[4] >> jtgoal[5];
                    jtgoal.Multiply(cmnPI_180);
                    jtposSet.SetGoal(jtgoal);
                    move_jp(jtposSet);
                    break;

                case 'M':   // position move Cartesian
                    GetCartesianPose(cartposSet);
                    if (cartposSet.GetMask().Any()) {
                        GetVersion(version);
                        if (version < 3) {
                            // Old versions of controller do not provide the target Cartesian
                            // position, so we instead use the measured position for elements
                            // that were not specified. In the future, this could be handled
                            // by mtsUniversalRobotScriptRT.
                            if (!cartposSet.GetMask()[0])
                                cartposSet.SetGoal(m_measured_cp.Position().Translation());
                            if (!cartposSet.GetMask()[1])
                                cartposSet.SetGoal(m_measured_cp.Position().Rotation());
                        }
                        move_cp(cartposSet);
                    }
                    break;

                case 'v':   // velocity move joint
                    std::cout << std::endl << "Enter joint velocities (deg/sec): ";
                    std::cin >> jtvel[0] >> jtvel[1] >> jtvel[2] >> jtvel[3] >> jtvel[4] >> jtvel[5];
                    jtvel.Multiply(cmnPI_180);
                    jtvelSet.SetGoal(jtvel);
                    servo_jv(jtvelSet);
                    break;

                case 'V':  // velocity move Cartesian
                    std::cout << std::endl << "Enter Cartesian XYZ velocities (mm/sec): ";
                    std::cin >> velxyz[0] >> velxyz[1] >> velxyz[2];
                    velxyz.Divide(1000.0);  // Convert from mm to m
                    std::cout << std::endl << "Enter Cartesian angular velocities (deg/sec): ";
                    std::cin >> velrot[0] >> velrot[1] >> velrot[2];
                    velrot.Multiply(cmnPI_180);
                    cartVelSet.SetTranslationGoal(velxyz);
                    cartVelSet.SetRotationGoal(velrot);
                    servo_cp(cartVelSet);
                    break;

                case 'd':  // display debug data
                    debugMode = !debugMode;
                    break;

                case 'c':  // Cartesian display mode
                    cartMode = !cartMode;
                    break;

                case 'D':
                    std::cout << std::endl << "Enter command: ";
                    std::getline(std::cin, buffer);
                    buffer.push_back('\n');
                    SendToDashboardServer(buffer);
                    break;

                case 'p':
                    std::cout << std::endl << "Enter payload (kg): ";
                    std::cin >> payload;
                    SetPayload(payload);
                    break;

                case 'P':
                    std::cout << std::endl << "Enter message: ";
                    std::getline(std::cin, buffer);
                    ShowPopup(buffer);
                    break;

                case 's':   // stop motion
                    StopMotion();
                    break;

                case 'x':   // get version
                    GetVersion(version);
                    if ((version < 0) || (version >= sizeof(versionString)/sizeof(versionString[0])))
                        std::cout << std::endl << "Firmware version, invalid response = " << version << std::endl;
                    else
                        std:: cout << std::endl << "Firmware version: " << versionString[version] << std::endl;
                    break;

                case 'f':   // free drive mode
                    SetRobotFreeDriveMode();
                    break;

                case 'r':   // running mode
                    SetRobotRunningMode();
                    break;

                case 'R':   // relative Cartesian motion
                    GetCartesianPose(cartposSet);
                    GetVersion(version);
                    if (version < 3) {
                        std::cout << std::endl
                                  << "Relative Cartesian motion not yet supported on CB2" << std::endl;
                    }
                    else if (cartposSet.GetMask().Any())
                        move_cr(cartposSet);
                    break;

                case 'i':   // robot info
                    std::cout << std::endl
                              << "Controller Time: " << cTime << std::endl
                              << "Controller Execution Time: " << cExecTime << std::endl;
                    if (GetRobotMode.IsValid()) {
                        GetRobotMode(robotMode);
                        GetVersion(version);
                        std::cout  << "Robot Mode: " << mtsUniversalRobotScriptRT::RobotModeName(robotMode, version) << std::endl;
                    }
                    if (GetSafetyMode.IsValid()) {
                        GetSafetyMode(safetyMode);
                        std::cout << "Safety Mode: " << mtsUniversalRobotScriptRT::SafetyModeName(safetyMode) << std::endl;
                    }
                    if (GetJointModes.IsValid()) {
                        GetJointModes(jointModes);
                        std::cout << "Joint Modes: ";
                        for (i = 0; i < jointModes.size(); i++)
                            std::cout << mtsUniversalRobotScriptRT::JointModeName(jointModes[i]) << " ";
                        std::cout << std::endl;
                    }
                    IsMotorPowerOn(flag);
                    if (flag)
                        std::cout << "Motor power is on" << std::endl;
                    else
                        std::cout << "Motor power is off" << std::endl;
                    IsEStop(flag);
                    if (flag)
                        std::cout << "Emergency stop is pressed" << std::endl;
                    else
                        std::cout << "Emergency stop is not pressed" << std::endl;
                    break;

                case 'e':   // enable motor power
                    EnableMotorPower();
                    break;

                case 'n':   // disable motor power
                    DisableMotorPower();
                    break;

                case 't':   // set tool voltage
                    std::cout << std::endl << "Enter tool voltage (0, 12, or 24): ";
                    std::cin >> toolVoltage;
                    SetToolVoltage(toolVoltage);
                    break;

                case 'q':   // quit program
                    std::cout << "Exiting.. " << std::endl;
                    this->Kill();
                    break;

                }
            }

            vctDoubleVec jtposDeg(m_measured_js.Position());
            jtposDeg.Multiply(cmn180_PI);
            if (debugMode) {
               printf("DEBUG: [%6.1lf,%6.1lf,%6.1lf,%6.1lf,%6.1lf,%6.1lf]                           \r",
                      debug[0], debug[1], debug[2], debug[3], debug[4], debug[5]);
            }
            else if (cartMode) {
                vct3 cpos(m_measured_cp.Position().Translation());
                cpos.Multiply(1000.0);  // Convert to mm
                vctEulerZYXRotation3 eulerRot(m_measured_cp.Position().Rotation());
                vct3 angles(eulerRot.GetAnglesInDegrees());
                printf("CART (mm, Euler deg): [ %7.2lf, %7.2lf, %7.2lf, %7.2lf, %7.2lf, %7.2lf ], PERIOD (s): %6.4lf\r",
                       cpos.X(), cpos.Y(), cpos.Z(), angles.X(), angles.Y(), angles.Z(), period);
            }
            else {
                printf("JOINTS (deg):         [ %7.2lf, %7.2lf, %7.2lf, %7.2lf, %7.2lf, %7.2lf ], PERIOD (s): %6.4lf\r",
                       jtposDeg[0], jtposDeg[1], jtposDeg[2], jtposDeg[3], jtposDeg[4], jtposDeg[5], period);
            }
        }
        osaSleep(0.01);  // to avoid taking too much CPU time
    }

    void Cleanup() {}

};

void UniversalRobotClient::GetCartesianPose(prmPositionCartesianSet &cartposSet)
{
    vct3 cartPos, cartVec;
    vctDoubleRot3 cartRot;

    std::cout << std::endl << "Enter Cartesian XYZ positions, mm (invalid char to skip): ";
    std::cin >> cartPos[0] >> cartPos[1] >> cartPos[2];
    if (std::cin.good()) {
        cartPos.Divide(1000.0);  // convert from mm to m
        cartposSet.SetGoal(cartPos);
    }
    else {
        std::cout << "Skipping position" << std::endl;
        std::cin.clear();
        std::cin.ignore(100, '\n');
    }
    std::cout << "Enter Cartesian Euler-ZYX orientation, deg (invalid char to skip): ";
    std::cin >> cartVec[0] >> cartVec[1] >> cartVec[2];
    if (std::cin.good()) {
        cartVec.Multiply(cmnPI_180);
        vctEulerZYXRotation3 rot(cartVec);
        cartRot.From(rot);
        cartposSet.SetGoal(cartRot);
    }
    else {
        std::cout << "Skipping orientation" << std::endl;
        std::cin.clear();
        std::cin.ignore(100, '\n');
    }
}

int main(int argc, char **argv)
{
    cmnLogger::SetMask(CMN_LOG_ALLOW_ALL);
    cmnLogger::SetMaskDefaultLog(CMN_LOG_ALLOW_ALL);
    cmnLogger::SetMaskFunction(CMN_LOG_ALLOW_ALL);
    cmnLogger::SetMaskClass("mtsUniversalRobotScriptRT", CMN_LOG_ALLOW_ALL);
    cmnLogger::AddChannel(std::cout, CMN_LOG_ALLOW_ERRORS_AND_WARNINGS);

    if (argc < 2) {
        std::cerr << "Syntax: sawUniversalRobotConsole <ip>" << std::endl;
        std::cerr << "        <ip>        IP address of Universal Robot" << std::endl;
        return 0;
    }
    mtsUniversalRobotScriptRT *urServer;
    urServer = new mtsUniversalRobotScriptRT("URserver");
    urServer->Configure(argv[1]);

    mtsComponentManager * componentManager = mtsComponentManager::GetInstance();
    componentManager->AddComponent(urServer);

    UniversalRobotClient client;
    componentManager->AddComponent(&client);

    if (!componentManager->Connect(client.GetName(), "Input", urServer->GetName(), "control")) {
        std::cout << "Failed to connect: "
            << client.GetName() << "::Input to "
            << urServer->GetName() << "::control" << std::endl;
        delete urServer;
        return -1;
    }

    componentManager->CreateAll();
    componentManager->StartAll();

    // Main thread passed to client task

    urServer->Kill();
    componentManager->WaitForStateAll(mtsComponentState::FINISHED, 2.0 * cmn_s);

    componentManager->Cleanup();

    // stop all logs
    cmnLogger::SetMask(CMN_LOG_ALLOW_NONE);
    delete urServer;

    return 0;
}
