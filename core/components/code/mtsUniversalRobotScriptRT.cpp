/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s): Peter Kazanzides, H. Tutkun Sen, Shuyang Chen

  (C) Copyright 2016-2024 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/

#include <stdio.h>
#include <stdlib.h>

#include <cisstCommon/cmnPortability.h>
#include <cisstCommon/cmnUnits.h>

#if (CISST_OS == CISST_WINDOWS)
typedef unsigned __int32 uint32_t;
#else
#include <stdint.h>
#endif

#include <cisstVector/vctRodriguezRotation3.h>
#include <cisstMultiTask/mtsInterfaceProvided.h>
#include <sawUniversalRobot/mtsUniversalRobotScriptRT.h>

const double MAX_VELOCITY = 12.0 * cmnPI_180;
const double MIN_VELOCITY = 0.001 * cmnPI_180;
const double MAX_ACCELERATION = 4.0 * cmnPI_180;

// Packet structs for different versions
#pragma pack(push, 1)     // Eliminate structure padding
struct module1 {
    uint32_t messageSize;
    double time;          // Time elapsed since controller was started
    double qTarget[6];    // Target joint positions
    double qdTarget[6];   // Target joint velocities
    double qddTarget[6];  // Target joint accelerations
    double I_Target[6];   // Target joint currents
    double M_Target[6];   // Target joint torques
    double qActual[6];    // Actual joint positions
    double qdActual[6];   // Actual joint velocities
    double I_Actual[6];   // Actual joint currents
};
#pragma pack(pop)

#pragma pack(push, 1)
struct module2 {
    unsigned long long digital_Input;  // Digital input bitmask
    double motor_Tem[6];      // Joint temperatures (degC)
    double controller_Time;   // Controller real-time thread execution time
    double test_Val;          // UR internal use only
    double robot_Mode;        // Robot mode (see RobotModes enum)
    double joint_Modes[6];    // Joint control modes (Version 1.8+, see JointModes enum)
};
#pragma pack(pop)

#pragma pack(push, 1)
struct packet_pre_3 {
    module1 base1;
    double tool_Accele[3];    // Tool accelerometer values (Version 1.7+)
    double blank[15];         // Unused
    double TCP_force[6];      // Generalized forces in the TCP
    double tool_Vector[6];    // Tool Cartesian pose (x, y, z, rx, ry, rz)
    double TCP_speed[6];      // Tool Cartesian speed
    module2 base2;
};
#pragma pack(pop)

#pragma pack(push, 1)
struct packet_30_31 {
    module1 base1;
    double I_ctrl[6];         // Joint control currents
    double tool_vec_Act[6];   // Actual tool Cartesian pose (x, y, z, rx, ry, rz)
    double TCP_speed_Act[6];  // Actual tool Cartesian speed
    double TCP_force[6];      // Generalized forces in the TCP
    double tool_vec_Tar[6];   // Target tool Cartesian pose (x, y, z, rx, ry, rz)
    double TCP_speed_Tar[6];  // Target tool Cartesian speed
    module2 base2;
    double safety_Mode;       // Safety mode
    double blank1[6];         // UR software only
    double tool_Accele[3];    // Tool accelerometer values
    double blank2[6];         // UR software only
    double speed_Scal;        // Speed scaling of trajectory limiter
    double linear_M_norm;     // Norm of Cartesian linear momentum
    double blank3;            // UR software only
    double blank4;            // UR software only
    double V_main;            // Masterboard main voltage
    double V_robot;           // Masterboard robot voltage (48V)
    double I_robot;           // Masterboard robot current
    double V_joint_Act[6];    // Actual joint voltages
};
#pragma pack(pop)

#pragma pack(push, 1)
// Versions 3.2-3.4
struct packet_32_34 : public packet_30_31 {
    unsigned long long digital_Output; // Digital outputs
    double program_State;     // Program state
};
#pragma pack(pop)

#pragma pack(push, 1)
// Versions 3.5-3.9 and 5.0-5.3
struct packet_35_39_50_53 : public packet_32_34 {
    double elbow_position[3];
    double elbow_velocity[3];
};
#pragma pack(pop)

#pragma pack(push, 1)
// Versions 3.10-3.13 and 5.4-5.8
struct packet_310_313_54_58 : public packet_35_39_50_53 {
    double safety_status;
};
#pragma pack(pop)

#pragma pack(push, 1)
// Versions 3.14-3.15 and 5.9
struct packet_314_315_59 : public packet_310_313_54_58 {
    double blank5[3];   // Used by Universal Robots software only
};
#pragma pack(pop)

#pragma pack(push, 1)
// Version 5.10
struct packet_510 : public packet_314_315_59 {
    double payload_mass;        // payload mass, kg
    double payload_cog[3];      // payload center of gravity, m
    double payload_inertia[6];  // Ixx, Iyy, Izz, Ixy, Ixz, Iyz, kg*m^2
};
#pragma pack(pop)

CMN_IMPLEMENT_SERVICES_DERIVED_ONEARG(mtsUniversalRobotScriptRT, mtsTaskContinuous, mtsTaskContinuousConstructorArg)

unsigned long mtsUniversalRobotScriptRT::PacketLength[VER_MAX] = {
       0,  // VER_UNKNOWN
     764,  // VER_PRE_18
     812,  // VER_18
    1044,  // VER_30_31
    1060,  // VER_32_34
    1108,  // VER_35_39   (or VER_50_53)
    1116,  // VER_310_313 (or VER_54_58)
    1140,  // VER_314_315 (or VER_59)
       0,  // VER_3_NEW
    1108,  // VER_50_53
    1116,  // VER_54_58
    1140,  // VER_59
    1220,  // VER_510
       0   // VER_5_NEW
};

char *mtsUniversalRobotScriptRT::VersionName[VER_MAX] = {
    "Unknown",
    "Pre-1.8",
    "1.8",
    "3.0-3.1",
    "3.2-3.4",
    "3.5-3.9",
    "3.10-3.13",
    "3.14-3.15",
    "3.+",
    "5.0-5.3",
    "5.4-5.8",
    "5.9",
    "5.10",
    "5.+"
};

// Static methods
std::string mtsUniversalRobotScriptRT::RobotModeName(int mode, int version)
{
    static const char *namesCB2[] = { "RUNNING", "FREEDRIVE", "READY", "INITIALIZING", "SECURITY_STOPPED",
                                      "EMERGENCY_STOPPED", "FAULT", "NO_POWER", "NOT_CONNECTED", "SHUTDOWN" };

    static const char *namesCB3[] = { "DISCONNECTED", "CONFIRM_SAFETY", "BOOTING", "POWER_OFF", "POWER_ON",
                                      "IDLE", "BACKDRIVE", "RUNNING", "UPDATING_FIRMWARE" };

    std::string str;
    if (mode == ROBOT_MODE_NO_CONTROLLER)
        str.assign("NO_CONTROLLER");
    else if ((version >= VER_PRE_18) && (version <= VER_18)) {
        // Controller Box 2 (CB2), also includes CB2.1
        if ((mode >= ROBOT_RUNNING_MODE) && (mode <= ROBOT_SHUTDOWN_MODE))
            str.assign(namesCB2[mode]);
        else
            str.assign("INVALID");
    }
    else if ((version >= VER_30_31) && (version < VER_MAX)) {
        // Controller Box 3 (CB3), also includes CB3.1, and 5 (e-Series)
        if ((mode >= ROBOT_MODE_DISCONNECTED) && (mode <= ROBOT_MODE_UPDATING_FIRMWARE))
            str.assign(namesCB3[mode]);
        else
            str.assign("INVALID");
    }
    else {
        str.assign("INVALID FIRMWARE VERSION");
    }
    return str;
}

// Note that "READY_FOR_POWER_OFF" is replaced by "EMERGENCY_STOPPED" for older versions
// (see Startup method). Previously, "VIOLATION" was called "SECURITY_STOPPED", but this
// seems to just be a terminology change.
char *mtsUniversalRobotScriptRT::JointModeNames[JOINT_MODE_IDLE-JOINT_MODE_RESET+1] =
           { "RESET", "SHUTTING_DOWN", "PART_D_CALIBRATION", "BACKDRIVE", "POWER_OFF",
             "READY_FOR_POWER_OFF", "CALVAL_INITIALIZATION", "ERROR",
             "FREEDRIVE", "SIMULATED", "NOT_RESPONDING", "MOTOR_INITIALISATION",
             "BOOTING", "PART_D_CALIBRATION_ERROR", "BOOTLOADER", "CALIBRATION",
             "VIOLATION", "FAULT", "RUNNING", "INITIALISATION", "IDLE" };

std::string mtsUniversalRobotScriptRT::JointModeName(int mode)
{
    std::string str;
    if ((mode >= JOINT_MODE_RESET) && (mode <= JOINT_MODE_IDLE))
        str.assign(JointModeNames[mode-JOINT_MODE_RESET]);
    else
        str.assign("INVALID");
    return str;
}

std::string mtsUniversalRobotScriptRT::SafetyModeName(int mode)
{
    static const char *names[] = { "UNKNOWN", "NORMAL", "REDUCED", "PROTECTIVE_STOP", "RECOVERY",
                                   "SAFEGUARD_STOP", "SYSTEM_EMERGENCY_STOP", "ROBOT_EMERGENCY_STOP",
                                   "VIOLATION", "FAULT", "VALIDATE_JOINT_ID", "UNDEFINED" };

    std::string str;
    if (mode <= SAFETY_MODE_UNDEFINED_SAFETY_MODE)
        str.assign(names[mode]);
    else
        str.assign("INVALID");
    return str;
}

// Constructor
mtsUniversalRobotScriptRT::mtsUniversalRobotScriptRT(const std::string &name, unsigned int sizeStateTable, bool newThread) :
    mtsTaskContinuous(name, sizeStateTable, newThread),
    ConfigurationStateTable(100, "Configuration"),
    buffer_idx(0), version(VER_UNKNOWN)
{
    Init();
}

mtsUniversalRobotScriptRT::mtsUniversalRobotScriptRT(const mtsTaskContinuousConstructorArg &arg) :
    mtsTaskContinuous(arg),
    ConfigurationStateTable(100, "Configuration"),
    buffer_idx(0), version(VER_UNKNOWN)
{
    Init();
}

mtsUniversalRobotScriptRT::~mtsUniversalRobotScriptRT()
{
    socket.Close();
    socketDB.Close();
}

void mtsUniversalRobotScriptRT::Init(void)
{
    UR_State = UR_NOT_CONNECTED;
    mShouldBeConnected = false;
    mTimeOfLastConnectAttempt = 0.0;
    socketDBconnected = false;

    ControllerTime = 0.0;
    ControllerExecTime = 0.0;
    robotMode = ROBOT_MODE_NO_CONTROLLER;
    jointModes.SetAll(0);  // Not a valid joint mode
    safetyMode = SAFETY_MODE_UNKNOWN;
    payload = 0.0;
    isPowerOn = false;
    isEStop = false;
    isSecurityStop = false;
    isMotionActive = false;

    pversion.major = 0;
    pversion.minor = 0;
    pversion.bugfix = 0;

    ticksPerSec = 125;        // Default for CB2/CB3
    expectedPeriod = 0.008;   // Default for CB2/CB3

    // Joint Configuration
    m_configuration_j.Name().resize(NB_Actuators);
    m_configuration_j.Name().at(0) = "shoulder_pan_joint";
    m_configuration_j.Name().at(1) = "shoulder_lift_joint";
    m_configuration_j.Name().at(2) = "elbow_joint";
    m_configuration_j.Name().at(3) = "wrist_1_joint";
    m_configuration_j.Name().at(4) = "wrist_2_joint";
    m_configuration_j.Name().at(5) = "wrist_3_joint";
    m_configuration_j.Type().SetSize(NB_Actuators);
    m_configuration_j.Type().SetAll(CMN_JOINT_REVOLUTE);

    // Actual joint state (measured values)
    cmnDataCopy(m_measured_js.Name(), m_configuration_j.Name());
    std::cerr << CMN_LOG_DETAILS << " need to add joint configuration with names, type and maybe limits?  Can we retrieve these via API?" << std::endl;
    m_measured_js.Position().SetSize(NB_Actuators, 0.0);
    m_measured_js.Velocity().SetSize(NB_Actuators, 0.0);
    m_measured_js.Effort().SetSize(NB_Actuators, 0.0);
    // Desired joint state (commanded values)
    cmnDataCopy(m_setpoint_js.Name(), m_configuration_j.Name());
    m_setpoint_js.Position().ForceAssign(m_measured_js.Position());
    m_setpoint_js.Velocity().ForceAssign(m_measured_js.Velocity());
    m_setpoint_js.Effort().ForceAssign(m_measured_js.Effort());
    // cartesian data
    TCPSpeed.SetAll(0.0);
    TCPForce.SetAll(0.0);
    setpointCP.SetAll(0.0);
    jtpos.SetSize(NB_Actuators);
    jtvel.SetSize(NB_Actuators);
    debug.SetAll(0.0);

    // Populate state tables
    StateTable.AddData(ControllerTime, "ControllerTime");
    StateTable.AddData(ControllerExecTime, "ControllerExecTime");
    StateTable.AddData(robotMode, "RobotMode");
    StateTable.AddData(jointModes, "JointModes");
    StateTable.AddData(safetyMode, "SafetyMode");
    StateTable.AddData(payload, "Payload");
    StateTable.AddData(isPowerOn, "IsPowerOn");
    StateTable.AddData(isEStop, "IsEStop");
    StateTable.AddData(isSecurityStop, "IsSecurityStop");
    StateTable.AddData(isMotionActive, "IsMotionActive");
    StateTable.AddData(m_measured_js, "measured_js");
    StateTable.AddData(m_setpoint_js, "setpoint_js");
    StateTable.AddData(m_measured_cp, "measured_cp");
    StateTable.AddData(m_setpoint_cp, "setpoint_cp");
    StateTable.AddData(TCPSpeed, "VelocityCartesian");
    StateTable.AddData(m_measured_cv, "measured_cv");
    StateTable.AddData(m_setpoint_cv, "setpoint_cv");
    StateTable.AddData(TCPForce, "ForceCartesianForce");
    StateTable.AddData(m_measured_cf, "measured_cf");
    StateTable.AddData(debug, "Debug");

    AddStateTable(&ConfigurationStateTable);
    ConfigurationStateTable.SetAutomaticAdvance(false);
    ConfigurationStateTable.Start();
    ConfigurationStateTable.Advance();
    ConfigurationStateTable.AddData(m_configuration_j, "configuration_j");

    mInterface = AddInterfaceProvided("control");
    if (mInterface) {
        // for Status, Warning and Error with mtsMessage
        mInterface->AddMessageEvents();

        // Standard interfaces (same as dVRK)
        mInterface->AddCommandReadState(this->StateTable, m_measured_js, "measured_js");
        mInterface->AddCommandReadState(this->StateTable, m_setpoint_js, "setpoint_js");
        mInterface->AddCommandReadState(this->StateTable, m_measured_cp, "measured_cp");
        mInterface->AddCommandReadState(this->StateTable, m_setpoint_cp, "setpoint_cp");
        mInterface->AddCommandReadState(this->StateTable, m_measured_cv, "measured_cv");
        mInterface->AddCommandReadState(this->StateTable, m_setpoint_cv, "setpoint_cv");
        mInterface->AddCommandReadState(this->StateTable, m_measured_cf, "measured_cf");
        mInterface->AddCommandWrite(&mtsUniversalRobotScriptRT::servo_jv,
                                    this, "servo_jv");
        mInterface->AddCommandWrite(&mtsUniversalRobotScriptRT::move_jp,
                                    this, "move_jp");
        mInterface->AddCommandWrite(&mtsUniversalRobotScriptRT::servo_cv,
                                    this, "servo_cv");
        mInterface->AddCommandWrite(&mtsUniversalRobotScriptRT::move_cp,
                                    this, "move_cp");
        mInterface->AddCommandWrite(&mtsUniversalRobotScriptRT::move_cr,
                                    this, "move_cr");

        mInterface->AddCommandReadState(this->ConfigurationStateTable, m_configuration_j, "configuration_js");

        // Following are not yet standardized
        mInterface->AddCommandReadState(StateTable, ControllerTime, "GetControllerTime");
        mInterface->AddCommandReadState(StateTable, ControllerExecTime, "GetControllerExecTime");
        mInterface->AddCommandReadState(StateTable, robotMode, "GetRobotMode");
        mInterface->AddCommandReadState(StateTable, jointModes, "GetJointModes");
        mInterface->AddCommandReadState(StateTable, safetyMode, "GetSafetyMode");
        mInterface->AddCommandReadState(StateTable, payload, "GetPayload");
        mInterface->AddCommandReadState(StateTable, isPowerOn, "IsMotorPowerOn");
        mInterface->AddCommandReadState(StateTable, isEStop, "IsEStop");
        mInterface->AddCommandReadState(StateTable, isSecurityStop, "IsSecurityStop");
        mInterface->AddCommandReadState(StateTable, isMotionActive, "IsMotionActive");
        mInterface->AddCommandVoid(&mtsUniversalRobotScriptRT::EnableMotorPower, this, "EnableMotorPower");
        mInterface->AddCommandVoid(&mtsUniversalRobotScriptRT::DisableMotorPower, this, "DisableMotorPower");
        mInterface->AddCommandVoid(&mtsUniversalRobotScriptRT::UnlockSecurityStop, this, "UnlockSecurityStop");
        mInterface->AddCommandRead(&mtsUniversalRobotScriptRT::GetConnected, this, "GetConnected");
        mInterface->AddCommandRead(&mtsUniversalRobotScriptRT::GetAveragePeriod, this, "GetAveragePeriod");
        mInterface->AddCommandVoid(&mtsUniversalRobotScriptRT::StopMotion, this, "StopMotion");
        mInterface->AddCommandVoid(&mtsUniversalRobotScriptRT::SetRobotRunningMode, this, "SetRobotRunningMode");
        mInterface->AddCommandVoid(&mtsUniversalRobotScriptRT::SetRobotFreeDriveMode, this, "SetRobotFreeDriveMode");
        mInterface->AddCommandReadState(StateTable, debug, "GetDebug");
        mInterface->AddCommandRead(&mtsUniversalRobotScriptRT::GetVersion, this, "GetVersion");
        mInterface->AddCommandRead(&mtsUniversalRobotScriptRT::GetVersionString, this, "GetVersionString");
        mInterface->AddCommandWrite(&mtsUniversalRobotScriptRT::SetGravity, this, "SetGravity");
        mInterface->AddCommandWrite(&mtsUniversalRobotScriptRT::SetPayload, this, "SetPayload");
        mInterface->AddCommandWrite(&mtsUniversalRobotScriptRT::SetToolFrame, this, "SetToolFrame");
        mInterface->AddCommandWrite(&mtsUniversalRobotScriptRT::SetToolVoltage, this, "SetToolVoltage");
        mInterface->AddCommandWrite(&mtsUniversalRobotScriptRT::ShowPopup, this, "ShowPopup");
        mInterface->AddCommandWrite(&mtsUniversalRobotScriptRT::SendToDashboardServer, this, "SendToDashboardServer");

        mInterface->AddEventVoid(SocketErrorEvent, "SocketError");
        mInterface->AddEventVoid(RobotNotReadyEvent, "RobotNotReady");
        mInterface->AddEventVoid(ReceiveTimeoutEvent, "ReceiveTimeout");
        vctULong2 arg;
        mInterface->AddEventWrite(PacketInvalid, "PacketInvalid", arg);

        // Stats
        mInterface->AddCommandReadState(StateTable, StateTable.PeriodStats,
                                        "GetPeriodStatistics");
        mInterface->AddCommandReadState(StateTable, StateTable.PeriodStats,
                                        "period_statistics");   // Used by mtsSystemQtWidget
    }

    for (size_t i = 0; i < VER_MAX; i++)
        PacketCount[i] = 0;
}

void mtsUniversalRobotScriptRT::Configure(const std::string &ipAddr)
{
    if (ipAddr.empty())
        CMN_LOG_CLASS_INIT_ERROR << "Configure: requires IP address" << std::endl;
    else {
        ipAddress = ipAddr;
        currentPort = 30003;
        CMN_LOG_CLASS_INIT_VERBOSE << "Configure: connecting to ip " << ipAddress
                                   << ", port " << currentPort << std::endl;
        if (socket.Connect(ipAddress.c_str(), currentPort)) {
            UR_State = UR_IDLE;
            mShouldBeConnected = true;
            CMN_LOG_CLASS_INIT_VERBOSE << "Configure: connected to port " << currentPort << std::endl;
        }
        else {
            CMN_LOG_CLASS_INIT_ERROR << "Configure: socket not connected" << std::endl;
        }
        if (socketDB.Connect(ipAddress.c_str(), 29999)) {
            socketDBconnected = true;
            CMN_LOG_CLASS_INIT_VERBOSE << "Configure: connected to Dashboard Server" << std::endl;
        }
        else {
            socketDBconnected = false;
            CMN_LOG_CLASS_INIT_ERROR << "Socket not connected to dashboard server" << std::endl;
        }
    }
}

void mtsUniversalRobotScriptRT::Startup(void)
{
    if (UR_State != UR_NOT_CONNECTED) {
        // Flush any existing packets
        while (socket.Receive(buffer, sizeof(buffer)) > 0);
        mInterface->SendStatus(this->GetName() + ": socket connected, IP: " + ipAddress);
    } else {
        mInterface->SendError(this->GetName() + ": socket not connected, IP: " + ipAddress);
    }
    ReadPolyscopeVersion();
    // JOINT_EMERGENCY_STOPPED_MODE was prior to Version 5.1
    if ((pversion.major < 5) || ((pversion.major == 5) && (pversion.minor == 0)))
        JointModeNames[JOINT_EMERGENCY_STOPPED_MODE] = "EMERGENCY_STOPPED";
}

void mtsUniversalRobotScriptRT::Run(void)
{
    m_measured_js.SetValid(false);
    m_setpoint_js.SetValid(false);
    m_measured_cp.SetValid(false);
    m_measured_cv.SetValid(false);
    m_measured_cf.SetValid(false);
    m_setpoint_cp.SetValid(false);
    m_setpoint_cv.SetValid(false);

    // Turn this on to enable sanity check of time difference.
    //bool timeCheckEnabled = (ControllerTime != 0);
    bool timeCheckEnabled = false;

    if (UR_State == UR_NOT_CONNECTED) {
        // Try to connect
        if (mShouldBeConnected) {
            if ((StateTable.Tic - mTimeOfLastConnectAttempt) > 30.0 * cmn_s) {
                if (socket.Connect(ipAddress.c_str(), currentPort)) {
                    UR_State = UR_IDLE;
                    mInterface->SendStatus(this->GetName() + ": reconnected, IP: " + ipAddress);
                } else {
                    mTimeOfLastConnectAttempt = StateTable.Tic;
                    mInterface->SendError(this->GetName() + ": failed to reconnect, IP: " + ipAddress);
                }
            }
        }
        // Call any connected components
        RunEvent();
        ProcessQueuedCommands();
        this->Sleep(expectedPeriod * cmn_s);
        return;
    }

    // Receive a packet with timeout. We choose a timeout of 1 sec, which is much
    // larger than expected (should get packets every 8 msec). Thus, if we don't get
    // a packet, then we raise the ReceiveTimeout event.
    buffer[buffer_idx+0] = buffer[buffer_idx+1] = buffer[buffer_idx+2] = buffer[buffer_idx+3] = 0;
    int numBytes = buffer_idx + socket.Receive(buffer+buffer_idx, sizeof(buffer)-buffer_idx, 1.0 * cmn_s);
    if (numBytes < 0) {
        buffer_idx = 0;
        SocketError();
        socket.Close();
        UR_State = UR_NOT_CONNECTED;
        RunEvent();
        ProcessQueuedCommands();
        return;
    }
    else if (numBytes == 0) {
        buffer_idx = 0;
        ReceiveTimeout();
        if ((StateTable.Tic - mTimeOfLastPacket) > 10.0 * cmn_s) {
            socket.Close();
            UR_State = UR_NOT_CONNECTED;
            mInterface->SendError(this->GetName() + ": no valid packet in past 10s, disconnecting IP: " + ipAddress);
            mTimeOfLastConnectAttempt = 0.0;
        }
        RunEvent();
        ProcessQueuedCommands();
        return;
    }
    mTimeOfLastPacket = StateTable.Tic;

    uint32_t packageLength;
    // Byteswap package length
    char *p = (char *)(&packageLength);
    p[0] = buffer[3]; p[1] = buffer[2]; p[2] = buffer[1]; p[3] = buffer[0];
    if ((packageLength > 0) && (packageLength <= static_cast<uint32_t>(numBytes))) {
        // Byteswap all the doubles in the package
        for (size_t i = 4; i < packageLength-4; i += 8) {
            for (size_t j = 0; j < 4; j++) {
                char tmp = buffer[i + j];
                buffer[i + j] = buffer[i + 7 - j];
                buffer[i + 7 - j] = tmp;
            }
        }
        // Check against expected versions. Even if we already know which version of firmware
        // we are communicating with, we keep checking in case we made a mistake. This also
        // collects useful debug data.
        int i;
        int vStart = VER_PRE_18;
        int vEnd = VER_MAX;
        if (pversion.major == 3) {
            vStart = VER_30_31;
            vEnd = VER_3_NEW;
        }
        else if (pversion.major == 5) {
            vStart = VER_50_53;
            vEnd = VER_5_NEW;
        }
        for (i = vStart; i < vEnd; i++) {
            if (packageLength == PacketLength[i]) {
                PacketCount[i]++;
                if (version == VER_UNKNOWN)
                    version = static_cast<FirmwareVersion>(i);
                else if (i != version) {
                    // Could we have auto-detected the wrong version?
                    if (PacketCount[i] > PacketCount[version]) {
                        CMN_LOG_CLASS_RUN_WARNING << "Switching from version " << version
                                                  << " to version " << i << std::endl;
                        version = static_cast<FirmwareVersion>(i);
                    }
                }
                // Expected period is 0.008 sec (125 Hz) for CB2/CB3 and
                // 0.002 (500 Hz) for e-Series
                ticksPerSec = (version < VER_50_53) ? 125 : 500;
                expectedPeriod = (version < VER_50_53) ? 0.008 : 0.002;
                break;
            }
        }
        if ((i == VER_3_NEW) || (i == VER_5_NEW)) {
            if (PacketLength[i] == 0) {
                CMN_LOG_CLASS_RUN_WARNING << "Found new version, packet length = "
                                          << packageLength << ", polyscope version = "
                                          << GetPolyscopeVersionString() << std::endl;
            }
            else if (packageLength != PacketLength[i]) {
                CMN_LOG_CLASS_RUN_WARNING << "Switching new version packet length from "
                                          << PacketLength[i] << " to "
                                          << packageLength << std::endl;
            }
            PacketLength[i] = packageLength;
        }
        else if (i == VER_MAX) {
            // If we didn't find a match above, increment the VER_UNKNOWN packet counter
            PacketCount[VER_UNKNOWN]++;
        }
        if (version != VER_UNKNOWN) {
            if (packageLength < PacketLength[version]) {
                debug[2] += 1;
                debug[3] = packageLength;
            }
            else if (packageLength > PacketLength[version]) {
                debug[4] += 1;
                debug[5] = packageLength;
            }
            // Following is valid for all versions
            module1 *base1 = reinterpret_cast<module1 *>(buffer);
            // First, do a sanity check on the packet. The new ControllerTime (base1->time)
            // should be about 0.008 seconds (CB2/CB3) or 0.002 seconds (e-Series) later than
            // the previous value.
            double timeDiff = base1->time - ControllerTime;
            debug[0] = 1000.0*timeDiff;
            if (timeCheckEnabled && ((timeDiff < (expectedPeriod/2)) || (timeDiff > (3*expectedPeriod)))) {
                // Could create a different event
                PacketInvalid(vctULong2(numBytes, packageLength));
                // Try to recover next time
                ControllerTime += expectedPeriod;
            }
            else {
                ControllerTime = base1->time;
                m_measured_js.Position().Assign(base1->qActual);
                m_setpoint_js.Position().Assign(base1->qTarget);
                m_measured_js.Velocity().Assign(base1->qdActual);
                m_setpoint_js.Velocity().Assign(base1->qdTarget);
                // Note that efforts are actually currents; should specify torque
                // instead of current, but UR does not provide measured torque
                // (i.e., provides I_Actual, I_Target, and M_Target, but not M_Actual).
                m_measured_js.Effort().Assign(base1->I_Actual);
                m_setpoint_js.Effort().Assign(base1->I_Target);

                m_measured_js.SetValid(true);
                m_setpoint_js.SetValid(true);
            }
            module2 *base2 = (version <= VER_18) ? (module2 *)(&((packet_pre_3 *)buffer)->base2)
                                                 : (module2 *)(&((packet_30_31 *)buffer)->base2);
            // Following is documented to be "controller realtime thread execution time"
            // Not sure what this is, or what are the units
            ControllerExecTime = base2->controller_Time;
            debug[1] = ControllerExecTime;

            if (base2->robot_Mode < 0.0)
                robotMode = static_cast<int>(base2->robot_Mode-0.5);
            else
                robotMode = static_cast<int>(base2->robot_Mode+0.5);

            // joint_Modes should always be positive
            for (i = 0; i < NB_Actuators; i++)
                jointModes[i] = static_cast<int>(base2->joint_Modes[i]+0.5);

            // We use the jointModes rather than robotMode to determine whether power is on because
            // the defined jointModes are consistent between firmware versions, whereas robotMode is not.
            // For jointModes, power is on if we are RUNNING, FREEDRIVE, INITIALISATION, or VIOLATION.
            // The following code handles joints in any combination of the above states (e.g., some joints
            // can be in JOINT_INITIALISATION_MODE while the rest of the joints are in JOINT_RUNNING_MODE).
            // Note that in cisstVector, addition is specialized as logical OR for boolean vectors.
            vctBool6 jointHasPower(jointModes.ElementwiseEqual(JOINT_MODE_RUNNING));
            jointHasPower.Add(jointModes.ElementwiseEqual(JOINT_FREEDRIVE_MODE));
            jointHasPower.Add(jointModes.ElementwiseEqual(JOINT_INITIALISATION_MODE));
            jointHasPower.Add(jointModes.ElementwiseEqual(JOINT_MODE_VIOLATION));
            isPowerOn = jointHasPower.All();

            double *tool_vec = 0;
            if (version < VER_30_31) {
                packet_pre_3 *packet = (packet_pre_3 *)(buffer);
                // Documentation does not specify whether tool_Vector or TCP_speed
                // are the actual or target Cartesian position or velocity.
                // We assume they are the actual (measured) position or velocity.
                tool_vec = packet->tool_Vector;
                TCPSpeed.Assign(packet->TCP_speed);
                TCPForce.Assign(packet->TCP_force);
                safetyMode = SAFETY_MODE_UNKNOWN;
                // Whether e-stop is pressed.
                // Could instead use (robotMode == ROBOT_EMERGENCY_STOPPED_MODE)
                isEStop = jointModes.Equal(JOINT_EMERGENCY_STOPPED_MODE);
                // Whether security stop is activated.
                // Could instead use (robotMode == ROBOT_SECURITY_STOPPED_MODE)
                isSecurityStop = jointModes.Equal(JOINT_SECURITY_STOPPED_MODE);
            }
            else if (version >= VER_30_31) {
                packet_30_31 *packet = (packet_30_31 *)(buffer);
                tool_vec = packet->tool_vec_Act;         // actual Cartesian position
                TCPSpeed.Assign(packet->TCP_speed_Act);  // actual Cartesian velocity
                TCPForce.Assign(packet->TCP_force);
                safetyMode = static_cast<int>(packet->safety_Mode+0.5);  // should always be positive
                // Whether e-stop is pressed
                isEStop = (safetyMode == SAFETY_MODE_ROBOT_EMERGENCY_STOP);
                isSecurityStop = (safetyMode == SAFETY_MODE_PROTECTIVE_STOP);
                // Convert target Cartesian position to setpoint_cp; also save it in setpointCP
                // for possible use by move_cp and move_cr.
                setpointCP.Assign(packet->tool_vec_Tar);
                vct3 position(packet->tool_vec_Tar);
                vct3 orientation(packet->tool_vec_Tar+3);
                vctRodriguezRotation3<double> rot(orientation);
                vctDoubleRot3 cartRot(rot);  // rotation matrix, from world frame to the end-effector frame
                vctFrm3 frm(cartRot, position);
                m_setpoint_cp.SetPosition(frm);
                m_setpoint_cp.SetValid(true);
                // Target Cartesian velocity
                m_setpoint_cv.SetVelocity(vct6(packet->TCP_speed_Tar));
                m_setpoint_cv.SetValid(true);
                if (version >= VER_510) {
                    packet_510 *packet = (packet_510 *)(buffer);
                    payload = packet->payload_mass;
                }
            }
            if (tool_vec) {
                vct3 position(tool_vec);
                vct3 orientation(tool_vec+3);
                vctRodriguezRotation3<double> rot(orientation);
                vctDoubleRot3 cartRot(rot);  // rotation matrix, from world frame to the end-effector frame
                vctFrm3 frm(cartRot, position);
                m_measured_cp.SetPosition(frm);
                m_measured_cp.SetValid(true);
            }
            m_measured_cv.SetVelocity(TCPSpeed);
            m_measured_cv.SetValid(true);
            m_measured_cf.SetForce(TCPForce);
            m_measured_cf.SetValid(true);
            // Finished with packet; now preserve any extra data for next time
            if (packageLength < static_cast<unsigned long>(numBytes)) {
                memmove(buffer, buffer+packageLength, numBytes-packageLength);
                buffer_idx = numBytes-packageLength;
            } else {
                buffer_idx = 0;
            }
        } else {
            buffer_idx = 0;
        }
    }
    else {
        buffer_idx = 0;
        PacketInvalid(vctULong2(numBytes, packageLength));
        mInterface->SendError(this->GetName() + ": invalid package");
        // purge buffer
        while (socket.Receive(buffer, sizeof(buffer)) > 0);
    }

    // Advance the state table now, so that any connected components can get
    // the latest data.
    StateTable.Advance();

    // Call any connected components
    RunEvent();

    ProcessQueuedCommands();

    isMotionActive = ((UR_State == UR_POS_MOVING) || (UR_State == UR_VEL_MOVING) ||
                      (UR_State == UR_FREE_DRIVE));

    switch (UR_State) {

    case UR_IDLE:
        break;

    case UR_VEL_MOVING:
        VelCmdTimeout--;
        if (VelCmdTimeout <= 0) {
            if (socket.Send(VelCmdStop) == -1)
                 SocketError();
            UR_State = UR_IDLE;
        }
        else {
            if (socket.Send(VelCmdString) == -1)
                 SocketError();
        }
        if (!isPowerOn)
            UR_State = UR_IDLE;
        break;

    case UR_FREE_DRIVE:
        if (!isPowerOn)
            UR_State = UR_IDLE;
        else if (version >= VER_30_31) {
            // Also using VelCmdTimeout, VelCmdString for free drive mode
            VelCmdTimeout--;
            if (VelCmdTimeout <= 0) {
                VelCmdTimeout = ticksPerSec;   // 1 second
                socket.Send(VelCmdString);
            }
        }
        break;

    case UR_POS_MOVING:
        // Motion is finished when target velocity is 0 or power is off
        if ((!m_setpoint_js.Velocity().Any()) || (!isPowerOn))
            UR_State = UR_IDLE;
        break;

    case UR_POWERING_ON:
        if (jointModes.Equal(JOINT_MODE_IDLE)) {
            if (version < VER_30_31) {
                // Seems to be necessary to send another "power on" command
                // before sending "brake release".
                if (socket.Send("power on\nbrake release\n") == -1)
                    SocketError();
            }
            else {
                // Starting with Version 3.0, send "brake release" command
                // via Dashboard Server.
                if (socketDB.Send("brake release\n") == -1)
                    SocketError();
            }
            UR_State = UR_IDLE;
        }
        else if (isEStop)
            UR_State = UR_IDLE;
        break;

    case UR_POWERING_OFF:
        break;

    default:
        CMN_LOG_CLASS_RUN_ERROR << "Run: unknown state = " << UR_State << std::endl;
    }

    // Check for any responses via Dashboard server
    if (socketDBconnected) {
        std::string response;
        if (RecvFromDashboardServer(response)) {
            mInterface->SendStatus(this->GetName() + "-DashboardServer: " + response);
        }
    }
}

void mtsUniversalRobotScriptRT::Cleanup(void)
{
}

void mtsUniversalRobotScriptRT::SocketError(void)
{
    SocketErrorEvent();
    mInterface->SendError(this->GetName() + ": socket error, IP: " + ipAddress);
}

void mtsUniversalRobotScriptRT::RobotNotReady(void)
{
    RobotNotReadyEvent();
    mInterface->SendWarning(this->GetName() + ": robot not ready");
}

void mtsUniversalRobotScriptRT::ReceiveTimeout(void)
{
    ReceiveTimeoutEvent();
    mInterface->SendError(this->GetName() + ": socket timeout");
}


bool mtsUniversalRobotScriptRT::SendAndReceiveDB(const std::string &cmd, std::string &recv)
{
    // First, flush any existing responses
    std::string response;
    while (RecvFromDashboardServer(response)) {
        mInterface->SendStatus(this->GetName() + "-DashboardServer: " + response);
    }
    // Now, send command
    if (socketDB.Send(cmd) < 0) return false;
    // Wait for reply
    this->Sleep(0.1);
    // Receive response
    return RecvFromDashboardServer(recv);
}

void mtsUniversalRobotScriptRT::SetRobotFreeDriveMode(void)
{
    if (isPowerOn && ((UR_State == UR_IDLE) || (UR_State == UR_FREE_DRIVE))) {
        if (version < VER_30_31) {
            if (socket.Send("set robotmode freedrive\n") == -1)
                SocketError();
        } else {
            VelCmdTimeout = 0;
            strcpy(VelCmdString, "def saw_ur_freedrive():\n\tfreedrive_mode()\n\tsleep(1.5)\nend\n");
            // This string will be sent from the Run method, once every 125 (CB2/CB3) or 500 (e-series)
            // loops (1 second). The programmed sleep is for 1.5 seconds, which should be long enough.
        }
        UR_State = UR_FREE_DRIVE;
        mInterface->SendStatus(this->GetName() + ": set freedrive mode");
    }
    else
        RobotNotReady();   // if not idle, ignore command and raise event
}

void mtsUniversalRobotScriptRT::SetRobotRunningMode(void)
{
    if (isPowerOn && (UR_State == UR_FREE_DRIVE)) {
        int ret;
        if (version < VER_30_31) {
            ret = socket.Send("set robotmode run\n");
        } else {
            ret = socket.Send("end_freedrive_mode()\n");
        }

        if (ret == -1) {
            SocketError();
        } else {
            UR_State = UR_IDLE;
            mInterface->SendStatus(this->GetName() + ": set running mode");
        }
    }
    else if (UR_State != UR_IDLE)
        RobotNotReady();
}

void mtsUniversalRobotScriptRT::EnableMotorPower(void)
{
    if (socket.Send("power on\n") == -1)
        SocketError();
    UR_State = UR_POWERING_ON;
}

void mtsUniversalRobotScriptRT::DisableMotorPower(void)
{
    if (socket.Send("power off\n") == -1)
        SocketError();
}

void mtsUniversalRobotScriptRT::UnlockSecurityStop(void)
{
    if (version >= VER_30_31) {
        // This code has been verified to work with Version 3.1. It is not known whether it works
        // with Version 3.0.  It does not work with Version 1.8.
        // Although not documented, it seems that sending "set unlock protective stop" via the script
        // interface works. An alternative would be to send "unlock protective stop" to the dashboard
        // server (socketDB), which is supported by Version 3.1+.
        if (socket.Send("set unlock protective stop\n") == -1)
            SocketError();
        // Although Version 3.1 introduced "close safety popup", that does not seem to close the
        // protective stop popup, whereas "close popup" works fine. Note that "close popup" is supported
        // starting with Version 1.6, but is not used for those older versions because there is no
        // benefit to closing the popup when the protective stop is still asserted.
        if (!socketDB.Send("close popup\n")) {
            CMN_LOG_CLASS_RUN_WARNING << "Failed to close popup" << std::endl;
        }
    }
    else
        mInterface->SendWarning(this->GetName() + ": UnlockSecurityStop not supported for this firmware version");
}

void mtsUniversalRobotScriptRT::servo_jv(const prmVelocityJointSet & jtvelSet)
{
    if (isPowerOn && ((UR_State == UR_IDLE) || (UR_State == UR_VEL_MOVING))) {
        jtvelSet.GetGoal(jtvel);
        // velocity check
        for (int i = 0; i < NB_Actuators; i++) {
            if (jtvel[i] > 0) {
                if (jtvel[i] < MIN_VELOCITY)
                    jtvel[i] = MIN_VELOCITY;
                else if (jtvel[i] > MAX_VELOCITY)
                    jtvel[i] = MAX_VELOCITY;
            }
            else {
                if (jtvel[i] > -MIN_VELOCITY)
                    jtvel[i] = -MIN_VELOCITY;
                else if (jtvel[i] < -MAX_VELOCITY)
                    jtvel[i] = -MAX_VELOCITY;
            }
        }
        // String length is about 28 + 6*7 = 70; set it to 100 to be sure
        // speedj(qd, a, t)
        // CB2 (firmware versions 1.8 or less) use speedj_init when the robot is initializing (homing).
        if ((version <= VER_18) && (robotMode == ROBOT_INITIALIZING_MODE)) {
            sprintf(VelCmdString,
                    "speedj_init([%6.4lf, %6.4lf, %6.4lf, %6.4lf, %6.4lf, %6.4lf], %6.4lf, 0.1)\n",
                    jtvel[0], jtvel[1], jtvel[2], jtvel[3], jtvel[4], jtvel[5], 1.4);
            strcpy(VelCmdStop, "speedj_init([0.0, 0.0, 0.0, 0.0, 0.0, 0.0], 1.4, 0.0)\n");
        }
        else {
            sprintf(VelCmdString,
                    "speedj([%6.4lf, %6.4lf, %6.4lf, %6.4lf, %6.4lf, %6.4lf], %6.4lf, 0.1)\n",
                    jtvel[0], jtvel[1], jtvel[2], jtvel[3], jtvel[4], jtvel[5], 1.4);
            strcpy(VelCmdStop, "speedj([0.0, 0.0, 0.0, 0.0, 0.0, 0.0], 1.4, 0.0)\n");
        }
        // Number of cycles for command to remain valid (1 second)
        VelCmdTimeout = ticksPerSec;
        UR_State = UR_VEL_MOVING;
    } else {
        RobotNotReady();
    }
}

void mtsUniversalRobotScriptRT::move_jp(const prmPositionJointSet & jtposSet)
{
    char JointPosCmdString[100];
    if (isPowerOn && (UR_State == UR_IDLE)) {
        jtposSet.GetGoal(jtpos);
        // For now, we issue a movej command; in the future, we may use a trajectory
        // generator and use servoj.
        // a (acceleration) is in rad/sec^2; v (velocity) is in rad/sec
        sprintf(JointPosCmdString,
                "movej([%6.4lf, %6.4lf, %6.4lf, %6.4lf, %6.4lf, %6.4lf], a=%6.4lf, v=%6.4lf)\n",
                jtpos[0], jtpos[1], jtpos[2], jtpos[3], jtpos[4], jtpos[5], 1.0, 0.2);
        if (socket.Send(JointPosCmdString) == -1) {
            SocketError();
        } else {
            UR_State = UR_POS_MOVING;
        }
    } else {
        RobotNotReady();
    }
}

void mtsUniversalRobotScriptRT::servo_cv(const prmVelocityCartesianSet & cartVel)
{
    if (isPowerOn && ((UR_State == UR_IDLE) || (UR_State == UR_VEL_MOVING))) {
        vct3 velxyz = cartVel.GetVelocity();
        vct3 velrot = cartVel.GetAngularVelocity();
        // a (acceleration) is in m/s^2
        double accel = cartVel.GetAcceleration().MaxAbsElement();
        if (accel == 0.0)
            accel = 1.4;   // default acceleration
        // speedl(qd, a, t, aRot)
        sprintf(VelCmdString,
                "speedl([%6.4lf, %6.4lf, %6.4lf, %6.4lf, %6.4lf, %6.4lf], %6.4lf, 0.1)\n",
                velxyz.X(), velxyz.Y(), velxyz.Z(), velrot.X(), velrot.Y(), velrot.Z(), accel);
        strcpy(VelCmdStop, "speedl([0.0, 0.0, 0.0, 0.0, 0.0, 0.0], 1.4, 0.0)\n");
        // Number of cycles for command to remain valid (1 second)
        VelCmdTimeout = ticksPerSec;
        UR_State = UR_VEL_MOVING;
    } else {
        RobotNotReady();
    }
}

void mtsUniversalRobotScriptRT::move_cp(const prmPositionCartesianSet & cartPos)
{
    char CartPosCmdString[100];
    if (isPowerOn && (UR_State == UR_IDLE)) {
        vctDoubleFrm3 cartFrm = cartPos.Goal();
        vctBool2 mask = cartPos.Mask();
        if (!mask.Any()) {
            CMN_LOG_CLASS_RUN_WARNING << "move_cp: no move specified (mask is false)" << std::endl;
            return;
        }
        if ((version < VER_30_31) && !mask.All()) {
            CMN_LOG_CLASS_RUN_ERROR << "move_cp: must specify both position and orientation for CB2" << std::endl;
            RobotNotReady();
            return;
        }
        vct3 posGoal;
        if (mask[0]) {
            posGoal = cartFrm.Translation();
        }
        else {
            posGoal.Assign(setpointCP.Pointer());
        }
        vct3 rotGoal;
        if (mask[1]) {
            vctRodriguezRotation3<double> rot;
            rot.From(cartFrm.Rotation());  // The rotation vector
            rotGoal.Assign(rot);
        }
        else {
            rotGoal.Assign(setpointCP.Pointer()+3);
        }
        // a (acceleration) is in m/s^2 and v (velocity) is in m/s.
        double trajVel = cartPos.Velocity().MaxAbsElement();
        if (trajVel == 0.0)
            trajVel = 0.03;  // default speed
        double trajAcc = cartPos.Acceleration().MaxAbsElement();
        if (trajAcc == 0.0)
            trajAcc = 0.8;   // default acceleration
        sprintf(CartPosCmdString,
            "movel(p[%6.4lf, %6.4lf, %6.4lf, %6.4lf, %6.4lf, %6.4lf], a=%6.4lf, v=%6.4lf)\n",
                posGoal.X(), posGoal.Y(), posGoal.Z(),
                rotGoal.X(), rotGoal.Y(), rotGoal.Z(), trajAcc, trajVel);
        if (socket.Send(CartPosCmdString) == -1) {
            SocketError();
        } else {
            UR_State = UR_POS_MOVING;
        }
    } else {
        RobotNotReady();
    }
}

void mtsUniversalRobotScriptRT::move_cr(const prmPositionCartesianSet & cartPos)
{
    vctBool2 mask = cartPos.Mask();
    if (!mask.Any()) {
        CMN_LOG_CLASS_RUN_WARNING << "move_cr: no move specified (mask is false)" << std::endl;
        return;
    }
    if (version < VER_30_31) {
        CMN_LOG_CLASS_RUN_ERROR << "move_cr: not currently supported on CB2" << std::endl;
        // May be able to use UR script function get_target_tcp_pose()
        RobotNotReady();
        return;
    }
    prmPositionCartesianSet cartPosAbs(cartPos);
    // Only need to update masked goals
    vct3 pos(m_setpoint_cp.Position().Translation());
    vctDoubleRot3 rot(m_setpoint_cp.Position().Rotation());
    if (mask[0]) {
        pos.Add(cartPos.Goal().Translation());
    }
    if (mask[1]) {
        rot = cartPos.Goal().Rotation()*rot;
    }
    cartPosAbs.SetGoal(vctDoubleFrm3(rot, pos));
    move_cp(cartPosAbs);
}

void mtsUniversalRobotScriptRT::StopMotion(void)
{
    if (socket.Send("stopj(1.4)\n") == -1)
        SocketError();
    else {
        if ((UR_State == UR_POS_MOVING) || (UR_State == UR_VEL_MOVING) || (UR_State == UR_FREE_DRIVE))
            UR_State = UR_IDLE;
    }
}

// Returns version detected from packet length (which is often a range of versions),
// followed by Polyscope version in parentheses.
void mtsUniversalRobotScriptRT::GetVersionString(std::string &str) const
{
    if (version < VER_MAX)
        str.assign(VersionName[version]);
    else
        str.assign("Invalid");
    if (pversion.major != 0) {
        str.append(" (");
        str.append(GetPolyscopeVersionString());
        str.append(")");
    }
}

// Set the gravity vector; in the script manual, this appears to be multiplied by "g" (9.82)
// rather than being a unit vector.
void mtsUniversalRobotScriptRT::SetGravity(const vct3 &gravity)
{
    char buf[100];
    sprintf(buf, "set_gravity([%8.3lf, %8.3lf, %8.3lf])\n", gravity.X(), gravity.Y(), gravity.Z());
    if (socket.Send(buf) == -1)
        SocketError();
}

// Set payload mass (kg). Robot can also accept a 3-vector for the center of mass.
// Note: starting with Version 3.3, the script interface also has set_payload_mass and set_payload_cog.
void mtsUniversalRobotScriptRT::SetPayload(const double &mass_kg)
{
    char buf[50];
    sprintf(buf, "set_payload(%8.3lf)\n", mass_kg);
    if (socket.Send(buf) == -1)
        SocketError();
    // Update the state table. Starting with Ver 5.10, the payload is returned via
    // the interface, which will overwrite this setting.
    payload = mass_kg;
}

// Set transformation from output flange to TCP
void mtsUniversalRobotScriptRT::SetToolFrame(const vctFrm3 &tcp)
{
    char buf[200];
    vctRodriguezRotation3<double> rot;
    rot.From(tcp.Rotation());
    sprintf(buf, "set_tcp(p[%8.3lf, %8.3lf, %8.3lf, %8.4lf, %8.4lf, %8.4lf])\n",
            tcp.Translation().X(), tcp.Translation().Y(), tcp.Translation().Z(),
            rot.X(), rot.Y(), rot.Z());
    if (socket.Send(buf) == -1)
        SocketError();
}

// Set power supply voltage at end effector (0, 12, or 24)
void mtsUniversalRobotScriptRT::SetToolVoltage(const int &voltage)
{
    char buf[100];
    if ((voltage == 0) || (voltage == 12) || (voltage == 24)) {
        sprintf(buf, "set_tool_voltage(%d)\n", voltage);
        if (socket.Send(buf) == -1)
            SocketError();
        else {
            sprintf(buf, ": setting tool voltage to %dV", voltage);
            mInterface->SendStatus(this->GetName() + buf);
        }
    }
    else {
        sprintf(buf, ": invalid voltage: %d", voltage);
        mInterface->SendWarning(this->GetName() + buf);
    }
}

// Show a popup message. The following parameters are left at their default values:
//    title ("popup"), warning (false), and error (true).
void mtsUniversalRobotScriptRT::ShowPopup(const std::string &msg)
{
    std::string buf("popup(\"" + msg + "\")\n");
    if (socket.Send(buf.c_str()) == -1)
        SocketError();
}

// Send the command to the Dashboard Server
void mtsUniversalRobotScriptRT::SendToDashboardServer(const std::string &msg)
{
    if (socketDB.Send(msg.c_str()) == -1)
        SocketError();
}

bool mtsUniversalRobotScriptRT::RecvFromDashboardServer(std::string &resp)
{
    char bufferDB[128];
    int nBytes = socketDB.Receive(bufferDB, sizeof(bufferDB));
    if (nBytes > 0) {
        bufferDB[nBytes-1] = 0;  // Remove last character (newline)
        resp.assign(bufferDB);
    }
    return (nBytes > 0);
}

bool mtsUniversalRobotScriptRT::ReadPolyscopeVersion(void)
{
    std::string pver;
    if (!SendAndReceiveDB("PolyscopeVersion\n", pver))
        return false;

    // According to e-Series manual, response has following format:
    //   "URSoftware 5.12.0.1101319 (Mar 22 2022)"
    // According to CB series manual, command has been available since
    // version 1.8 and response has following format: "3.0.15547", but
    // at least some CB3 series controllers return the longer message,
    // e.g., "URSoftware 3.12.0.90886 (Nov 15 2019)"
    CMN_LOG_CLASS_RUN_VERBOSE << "ReadPolyscopeVersion: " << pver << std::endl;

    // First, check for "URSoftware " string and advance past it if found
    std::string compareStr("URSoftware ");
    size_t offset = 0;
    if (pver.compare(0, compareStr.size(), compareStr) == 0)
        offset = compareStr.size();

    // Using sscanf instead of std::string and/or std::stringstream to parse version string
    int major, minor, bugfix;
    int n = sscanf(pver.c_str()+offset, "%d.%d.%d", &major, &minor, &bugfix);
    if (n == 3) {
        pversion.major = major;
        pversion.minor = minor;
        pversion.bugfix = bugfix;
        CMN_LOG_CLASS_RUN_WARNING << "ReadPolyscopeVersion: version = "
                                  << GetPolyscopeVersionString() << std::endl;
    }
    else {
        CMN_LOG_CLASS_RUN_WARNING << "ReadPolyscopeVersion: failed to parse version from " << pver << std::endl;
    }
    return (n == 3);
}

std::string mtsUniversalRobotScriptRT::GetPolyscopeVersionString(void) const
{
    std::ostringstream resp;
    resp << pversion.major << "." << pversion.minor << "." << pversion.bugfix;
    return resp.str();
}
