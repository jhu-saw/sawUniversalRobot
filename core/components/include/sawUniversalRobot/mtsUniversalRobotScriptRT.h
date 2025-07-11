/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s): Peter Kazanzides, H. Tutkun Sen, Shuyang Chen

  (C) Copyright 2016-2023 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/

#ifndef _mtsUniversalRobotScriptRT_h
#define _mtsUniversalRobotScriptRT_h

#include <cisstCommon/cmnPortability.h>
#include <cisstVector/vctTypes.h>
#include <cisstOSAbstraction/osaSocket.h>
#include <cisstMultiTask/mtsTaskContinuous.h>

#include <cisstParameterTypes/prmStateJoint.h>
#include <cisstParameterTypes/prmConfigurationJoint.h>
#include <cisstParameterTypes/prmPositionJointSet.h>
#include <cisstParameterTypes/prmVelocityJointSet.h>

#include <cisstParameterTypes/prmPositionCartesianGet.h>
#include <cisstParameterTypes/prmPositionCartesianSet.h>
#include <cisstParameterTypes/prmVelocityCartesianGet.h>
#include <cisstParameterTypes/prmVelocityCartesianSet.h>
#include <cisstParameterTypes/prmForceCartesianGet.h>

// Always include last
#include <sawUniversalRobot/sawUniversalRobotExport.h>

#ifdef CISST_COMPILER_IS_MSVC
// Disable warning about unsafe use of type 'bool' (in cisstVector)
#pragma warning(disable: 4804)
#endif

class CISST_EXPORT mtsUniversalRobotScriptRT : public mtsTaskContinuous
{
    CMN_DECLARE_SERVICES(CMN_DYNAMIC_CREATION_ONEARG, CMN_LOG_ALLOW_VERBOSE)

public:

    // RobotModes: definition varies depending on Controller Box version. Fortunately, it is possible
    // to distinguish versions from the firmware version: CB2 (and CB2.1) uses firmware up to Version 1.8,
    // whereas CB3 (and CB3.1) uses firmware starting with Version 3.0.
    // CB1: Not supported by this software (might be same as CB2)
    // CB2: Defined in Dashboard Server how-to; also in C API for Version 1.8, file robotinterface.h
    // CB3 (and e-Series): Defined in Client_Interface.xlsx.
    // For all versions, ROBOT_MODE_NO_CONTROLLER = -1
    enum RobotModesCB2 { ROBOT_RUNNING_MODE, ROBOT_FREEDRIVE_MODE,
                         ROBOT_READY_MODE, ROBOT_INITIALIZING_MODE, ROBOT_SECURITY_STOPPED_MODE,
                         ROBOT_EMERGENCY_STOPPED_MODE, ROBOT_FAULT_MODE, ROBOT_NO_POWER_MODE,
                         ROBOT_NOT_CONNECTED_MODE, ROBOT_SHUTDOWN_MODE };
    enum RobotModesCB3 { ROBOT_MODE_DISCONNECTED, ROBOT_MODE_CONFIRM_SAFETY, ROBOT_MODE_BOOTING,
                         ROBOT_MODE_POWER_OFF, ROBOT_MODE_POWER_ON, ROBOT_MODE_IDLE,
                         ROBOT_MODE_BACKDRIVE, ROBOT_MODE_RUNNING, ROBOT_MODE_UPDATING_FIRMWARE };
    enum { ROBOT_MODE_NO_CONTROLLER = -1 };

    static std::string RobotModeName(int mode, int version);

    // JointModes available starting with firmware version 1.8. Some of these definitions are not
    // documented in the script interface (Client_Interface.xlsx), but were instead obtained from
    // the C API (microprocessor_commands.h) for Version 1.8.
    // Note that this enum intentionally contains some duplicate values, since there is a single
    // enum for all versions.
    enum JointModes { JOINT_MODE_RESET = 235,
                      JOINT_MODE_SHUTTING_DOWN = 236,
                      JOINT_PART_D_CALIBRATION_MODE = 237,       // Internal use only
                      JOINT_MODE_BACKDRIVE = 238,
                      JOINT_MODE_POWER_OFF = 239,
                      JOINT_MODE_READY_FOR_POWER_OFF = 240,      // Version 5.1+
                      JOINT_EMERGENCY_STOPPED_MODE = 240,        // C API
                      JOINT_CALVAL_INITIALIZATION_MODE = 241,    // C API
                      JOINT_ERROR_MODE = 242,                    // C API
                      JOINT_FREEDRIVE_MODE = 243,                // C API
                      JOINT_SIMULATED_MODE = 244,                // C API
                      JOINT_MODE_NOT_RESPONDING = 245,
                      JOINT_MODE_MOTOR_INITIALISATION = 246,
                      JOINT_MODE_BOOTING = 247,
                      JOINT_PART_D_CALIBRATION_ERROR_MODE = 248, // Internal use only
                      JOINT_MODE_BOOTLOADER = 249,
                      JOINT_CALIBRATION_MODE = 250,              // Internal use only
                      JOINT_MODE_VIOLATION = 251,
                      JOINT_SECURITY_STOPPED_MODE = 251,         // Seems to now be called VIOLATION
                      JOINT_MODE_FAULT = 252,
                      JOINT_MODE_RUNNING = 253,
                      JOINT_INITIALISATION_MODE = 254,           // C API
                      JOINT_MODE_IDLE = 255 };

    static std::string JointModeName(int mode);

    // SafetyMode is available starting with firmware version 3.0. Since the value of 0 is not used by
    // Universal Robots, we define it to be SAFETY_MODE_UNKNOWN.
    // e-Series added SAFETY_MODE_VALIDATE_JOINT_ID (10)
    // Recent documentation added SAFETY_MODE_UNDEFINED_SAFETY_MODE (11)
    enum SafetyModes { SAFETY_MODE_UNKNOWN, SAFETY_MODE_NORMAL, SAFETY_MODE_REDUCED,
                       SAFETY_MODE_PROTECTIVE_STOP, SAFETY_MODE_RECOVERY,
                       SAFETY_MODE_SAFEGUARD_STOP,          // Physical s-stop interface input
                       SAFETY_MODE_SYSTEM_EMERGENCY_STOP,   // Physical e-stop interface input
                       SAFETY_MODE_ROBOT_EMERGENCY_STOP,    // Physical e-stop interface input
                       SAFETY_MODE_VIOLATION, SAFETY_MODE_FAULT,
                       SAFETY_MODE_VALIDATE_JOINT_ID,
                       SAFETY_MODE_UNDEFINED_SAFETY_MODE};

    static std::string SafetyModeName(int mode);

protected:
    enum {NB_Actuators = 6};

    enum UR_STATES { UR_NOT_CONNECTED, UR_IDLE, UR_POS_MOVING, UR_VEL_MOVING, UR_FREE_DRIVE, UR_POWERING_OFF, UR_POWERING_ON };
    UR_STATES UR_State;

    mtsStateTable ConfigurationStateTable;

    // This buffer must be large enough for largest packet size.
    // According to documentation, port 30003 packets are up to 1140 bytes (Version 3.15)
    // (On port 30001, have seen packets as large as 1295 bytes).
    // We keep it less than twice the minimum packet length (764 bytes) so that we cannot
    // accumulate more than one complete packet in the buffer.
    char buffer[1500];
    unsigned int buffer_idx;

    struct PolyScopeVersion {
        int major;
        int minor;
        int bugfix;
    } pversion;

    // Ticks per second (frequency) as an integer
    unsigned int ticksPerSec;
    // Expected period (0.008 for CB2/CB3, 0.002 for e-Series)
    double expectedPeriod;

    // State table entries
    double ControllerTime;
    double ControllerExecTime;
    int robotMode;
    vctInt6 jointModes;
    int safetyMode;
    double payload;    // payload mass in kg
    bool isPowerOn;
    bool isEStop;
    bool isSecurityStop;
    bool isMotionActive;

    prmConfigurationJoint m_configuration_j;
    prmStateJoint m_measured_js, m_setpoint_js;

    prmPositionCartesianGet m_measured_cp; // Actual Cartesian position (standard payload)
    vct6 setpointCP;                       // Cartesian setpoint (x,y,z,rx,ry,rz)
    prmPositionCartesianGet m_setpoint_cp; // Target Cartesian position (Ver 3+)
    vct6 TCPSpeed;                         // Actual Cartesian velocity
    prmVelocityCartesianGet m_measured_cv; // Actual Cartesian velocity (standard payload)
    prmVelocityCartesianGet m_setpoint_cv; // Target Cartesian velocity (Ver 3+)

    vct6 TCPForce;                         // Actual Cartesian force/torque
    prmForceCartesianGet m_measured_cf;    // Actual Cartesian force/torque (standard payload)

    // Internal use
    vctDoubleVec jtpos;
    vctDoubleVec jtvel;
    char VelCmdString[100];
    char VelCmdStop[100];
    int  VelCmdTimeout;

    // For real-time debugging
    vct6 debug;

    // For UR version determination
    enum FirmwareVersion {VER_UNKNOWN, VER_PRE_18, VER_18, VER_30_31, VER_32_34, VER_35_39, VER_310_313,
                          VER_314_315, VER_3_NEW, VER_50_53, VER_54_58, VER_59, VER_510, VER_5_NEW, VER_MAX};
    FirmwareVersion version;
    static char *VersionName[VER_MAX];
    static unsigned long PacketLength[VER_MAX];
    unsigned long PacketCount[VER_MAX];

    static char *JointModeNames[JOINT_MODE_IDLE-JOINT_MODE_RESET+1];

    // Called by constructors
    void Init(void);

    // Methods for provided interface

    // Enable motor power
    void EnableMotorPower(void);
    // Disable motor power
    void DisableMotorPower(void);

    // Unlock after security (protective) stop
    void UnlockSecurityStop(void);

    // Set Robot Modes
    void SetRobotFreeDriveMode(void);
    void SetRobotRunningMode(void);

    // Stop Motion
    void StopMotion(void);

    // Move joint at specified velocity (radians/sec)
    void servo_jv(const prmVelocityJointSet & jtvel);

    // Move joint to specified position (radians)
    void move_jp(const prmPositionJointSet & jtpos);

    // Cartesian velocity move
    void servo_cv(const prmVelocityCartesianSet & cartVel);

    // Cartesian position move
    void move_cp(const prmPositionCartesianSet & cartPos);

    // Cartesian position relative (incremental) move
    void move_cr(const prmPositionCartesianSet & cartPos);

    // Return the average period (measured by StateTable)
    void GetAveragePeriod(double &period) const
    { period = mtsTask::GetAveragePeriod(); }

    void GetConnected(bool &conn) const
    { conn = (UR_State != UR_NOT_CONNECTED); }

    void GetVersion(int &ver) const
    {
        ver = static_cast<int>(version);
    }

    void GetVersionString(std::string &str) const;

    // Set the gravity vector
    void SetGravity(const vct3 &gravity);

    // Set payload mass (kg). Robot can also accept a 3-vector
    // for the center of mass.
    // The payload can be read by GetPayload, but note that for
    // Versions prior to 5.10, this just returns the last value
    // written by SetPayload (i.e., it is not aware of any payload
    // changes made outside this interface).
    void SetPayload(const double &mass_kg);

    // Set transformation from output flange to TCP
    void SetToolFrame(const vctFrm3 &tcp);

    // Set tool (power supply) voltage; valid values are 0, 12, or 24.
    void SetToolVoltage(const int &voltage);

    // Display a popup message on the robot pendant
    void ShowPopup(const std::string &msg);

    // Send command to Dashboard Server
    void SendToDashboardServer(const std::string &msg);

    // Receive response from Dashboard Server
    bool RecvFromDashboardServer(std::string &resp);

    // Reads Polyscope version from Dashboard Server and stores
    // result in class
    bool ReadPolyscopeVersion(void);

    // Returns formatted string based on Polyscope version previously
    // obtained from Dashboard Server.
    std::string GetPolyscopeVersionString(void) const;

    // Connection Parameters
    // IP address (TCP/IP)
    std::string ipAddress;
    // UR ports:
    //    30001  - primary client (10 Hz)
    //    30002  - secondary client (10 Hz)
    //    30003  - real-time client (125 Hz)
    //    30004  - RTDE port (125 Hz)
    // Currently, only port 30003 is supported
    unsigned short currentPort;
    // Socket to UR controller
    osaSocket socket;
    // Flag used to indicate that robot used to be connected and needs to reconnect
    bool mShouldBeConnected;

    // Socket to Dashboard server (port 29999)
    osaSocket socketDB;
    bool socketDBconnected;

    // Time since last
    double mTimeOfLastPacket;

    // Time between reconnection attempts
    double mTimeOfLastConnectAttempt;

    // Event generators
    mtsFunctionVoid SocketErrorEvent;
    void SocketError(void);
    mtsFunctionVoid RobotNotReadyEvent;
    void RobotNotReady(void);
    mtsFunctionVoid ReceiveTimeoutEvent;
    void ReceiveTimeout(void);

    // return TRUE on success, FALSE on fail
    bool SendAndReceiveDB(const std::string &cmd, std::string &recv);

    mtsFunctionWrite PacketInvalid;
    mtsInterfaceProvided * mInterface;

public:

    mtsUniversalRobotScriptRT(const std::string &name, unsigned int sizeStateTable = 256, bool newThread = true);

    mtsUniversalRobotScriptRT(const mtsTaskContinuousConstructorArg &arg);

    virtual ~mtsUniversalRobotScriptRT();

    void Configure(const std::string &ipAddr = "");

    void Startup(void);

    void Run(void);

    void Cleanup(void);
};

CMN_DECLARE_SERVICES_INSTANTIATION(mtsUniversalRobotScriptRT)

#endif
