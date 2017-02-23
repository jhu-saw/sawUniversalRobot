/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s): Peter Kazanzides, H. Tutkun Sen, Shuyang Chen

  (C) Copyright 2016-2017 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/

#ifndef _mtsUniversalRobotScriptRT_h
#define _mtsUniversalRobotScriptRT_h

#include <cisstVector/vctTypes.h>
#include <cisstOSAbstraction/osaSocket.h>
#include <cisstMultiTask/mtsTaskContinuous.h>
#include <cisstParameterTypes/prmPositionJointGet.h>
#include <cisstParameterTypes/prmPositionJointSet.h>
#include <cisstParameterTypes/prmPositionCartesianGet.h>
#include <cisstParameterTypes/prmPositionCartesianSet.h>
#include <cisstParameterTypes/prmVelocityJointGet.h>
#include <cisstParameterTypes/prmVelocityJointSet.h>
#include <cisstParameterTypes/prmVelocityCartesianGet.h>
#include <cisstParameterTypes/prmVelocityCartesianSet.h>
#include <cisstParameterTypes/prmForceCartesianGet.h>

// Always include last
#include <sawUniversalRobot/sawUniversalRobotExport.h>

void CISST_EXPORT PrintPacketDebug(void);

class CISST_EXPORT mtsUniversalRobotScriptRT : public mtsTaskContinuous
{
    CMN_DECLARE_SERVICES(CMN_DYNAMIC_CREATION_ONEARG, CMN_LOG_ALLOW_VERBOSE)

protected:

    enum {NB_Actuators = 6};

    enum UR_STATES { UR_NOT_CONNECTED, UR_IDLE, UR_POS_MOVING, UR_VEL_MOVING, UR_FREE_DRIVE, UR_POWERING_OFF, UR_POWERING_ON };
    UR_STATES UR_State;

    enum RobotModes { ROBOT_MODE_DISCONNECTED, ROBOT_MODE_CONFIRM_SAFETY, ROBOT_MODE_BOOTING,
                      ROBOT_MODE_POWER_OFF, ROBOT_MODE_POWER_ON, ROBOT_MODE_IDLE,
                      ROBOT_MODE_BACKDRIVE, ROBOT_MODE_RUNNING, ROBOT_MODE_UPDATING_FIRMWARE };

    enum ControlModes { CONTROL_MODE_POSITION, CONTROL_MODE_TEACH, CONTROL_MODE_FORCE, CONTROL_MODE_TORQUE };

    enum JointModes { JOINT_SHUTTING_DOWN_MODE=236, JOINT_PART_D_CALIBRATION_MODE,
                      JOINT_BACKDRIVE_MODE, JOINT_POWER_OFF_MODE, JOINT_NOT_RESPONDING_MODE,
                      JOINT_MOTOR_INITIALISATION_MODE, JOINT_BOOTING_MODE,
                      JOINT_PART_D_CALIBRATION_ERROR_MODE, JOINT_BOOTLOADER_MODE,
                      JOINT_CALIBRATION_MODE, JOINT_FAULT_MODE, JOINT_RUNNING_MODE, JOINT_IDLE_MODE };

    // This buffer must be large enough for largest packet size.
    // According to documentation, port 30003 packets are up to 1060 bytes (Version 3.2)
    // (On port 30001, have seen packets as large as 1295 bytes).
    // We keep it less than twice the minimum packet length (764 bytes) so that we cannot
    // accumulate more than one complete packet in the buffer.
    char buffer[1500];
    unsigned int buffer_idx;

    // Packet structs for different versions
#pragma pack(push, 1)     // Eliminate structure padding
    struct module1 {
        unsigned int messageSize;
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
    struct packet_32 : packet_30_31 {
        unsigned long long digital_Output; // Digital outputs
        double program_State;     // Program state
    };
#pragma pack(pop)

    // State table entries
    double ControllerTime;
    double ControllerExecTime;

    vctDoubleVec JointPos;                // Actual joint position
    prmPositionJointGet JointPosParam;    // Actual joint position (standard payload)
    vctDoubleVec JointTargetPos;          // Desired joint position (feedback)

    vctDoubleVec JointVel;                // Actual joint velocity
    prmVelocityJointGet JointVelParam;    // Actual joint velocity (standard payload)
    vctDoubleVec JointTargetVel;          // Desired joint velocity (feedback)

    prmPositionCartesianGet CartPos;      // Actual Cartesian position (standard payload)
    vct6 TCPSpeed;                        // Actual Cartesian velocity
    prmVelocityCartesianGet CartVelParam; // Actual Cartesian velocity (standard payload)

    vct6 TCPForce;                        // Actual Cartesian force/torque
    prmForceCartesianGet WrenchGet;       // Actual Cartesian force/torque (standard payload)

    // Internal use
    char VelCmdString[100];
    char VelCmdStop[100];
    int  VelCmdTimeout;

    // For real-time debugging
    vct6 debug;

    // For UR version determination
    enum FirmwareVersion {VER_UNKNOWN, VER_PRE_18, VER_18, VER_30_31, VER_32, VER_MAX};
    FirmwareVersion version;
    static unsigned long PacketLength[VER_MAX];
    unsigned long PacketCount[VER_MAX];

    // Called by constructors
    void Init(void);

    // Methods for provided interface

    // Disable motor power
    void DisableMotorPower(void);

    // Set Robot Modes
    void SetRobotFreeDriveMode(void);
    void SetRobotRunningMode(void);

    // Stop Motion
    void StopMotion(void);

    // Move joint at specified velocity (radians/sec)
    void JointVelocityMove(const prmVelocityJointSet &jtvel);

    // Move joint to specified position (radians)
    void JointPositionMove(const prmPositionJointSet &jtpos);

    // Cartesian velocity move
    void CartesianVelocityMove(const prmVelocityCartesianSet &cartVel);

    // Cartesian position move
    void CartesianPositionMove(const prmPositionCartesianSet &cartPos);

    // Return the average period (measured by StateTable)
    void GetAveragePeriod(double &period) const
    { period = mtsTask::GetAveragePeriod(); }

    void GetConnected(bool &conn) const
    { conn = (UR_State != UR_NOT_CONNECTED); }

    void GetVersion(int &ver) const
    { ver = static_cast<int>(version); }

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

    // Event generators
    mtsFunctionVoid SocketErrorEvent;
    void SocketError(void);
    mtsFunctionVoid RobotNotReady;
    mtsFunctionVoid ReceiveTimeout;
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
