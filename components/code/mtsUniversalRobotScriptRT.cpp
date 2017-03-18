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

#include <stdio.h>
#include <stdlib.h>

#include <cisstCommon/cmnPortability.h>

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
struct packet_32 : packet_30_31 {
    unsigned long long digital_Output; // Digital outputs
    double program_State;     // Program state
};
#pragma pack(pop)

CMN_IMPLEMENT_SERVICES_DERIVED_ONEARG(mtsUniversalRobotScriptRT, mtsTaskContinuous, mtsTaskContinuousConstructorArg)

unsigned long mtsUniversalRobotScriptRT::PacketLength[VER_MAX] = {
       0,  // VER_UNKNOWN
     764,  // VER_PRE_18
     812,  // VER_18
    1044,  // VER_30_31
    1060   // VER_32
};

mtsUniversalRobotScriptRT::mtsUniversalRobotScriptRT(const std::string &name, unsigned int sizeStateTable, bool newThread) :
    mtsTaskContinuous(name, sizeStateTable, newThread), buffer_idx(0), version(VER_UNKNOWN)
{
    Init();
}

mtsUniversalRobotScriptRT::mtsUniversalRobotScriptRT(const mtsTaskContinuousConstructorArg &arg) :
    mtsTaskContinuous(arg), buffer_idx(0), version(VER_UNKNOWN)
{
    Init();
}

mtsUniversalRobotScriptRT::~mtsUniversalRobotScriptRT()
{
    socket.Close();
}

void mtsUniversalRobotScriptRT::Init(void)
{
    UR_State = UR_NOT_CONNECTED;
    mShouldBeConnected = false;
    mTimeOfLastConnectAttempt = 0.0;

    ControllerTime = 0.0;
    ControllerExecTime = 0.0;
    JointPos.SetSize(NB_Actuators);
    JointPos.SetAll(0.0);
    JointPosParam.SetSize(NB_Actuators);
    JointTargetPos.SetSize(NB_Actuators);
    JointTargetPos.SetAll(0.0);
    JointVel.SetSize(NB_Actuators);
    JointVel.SetAll(0.0);
    JointVelParam.SetSize(NB_Actuators);
    JointTargetVel.SetSize(NB_Actuators);
    JointTargetVel.SetAll(0.0);
    JointEffort.SetSize(NB_Actuators);
    JointEffort.SetAll(0.0);
    JointTargetEffort.SetSize(NB_Actuators);
    JointTargetEffort.SetAll(0.0);

    JointState.Name().SetSize(NB_Actuators);
    JointState.Name()[0] = "shoulder_pan_joint";
    JointState.Name()[1] = "shoulder_lift_joint";
    JointState.Name()[2] = "elbow_joint";
    JointState.Name()[3] = "wrist_1_joint";
    JointState.Name()[4] = "wrist_2_joint";
    JointState.Name()[5] = "wrist_3_joint";
    JointState.Position().ForceAssign(JointPos);
    JointState.Velocity().ForceAssign(JointVel);
    JointState.Effort().ForceAssign(JointEffort);
    TCPSpeed.SetAll(0.0);
    TCPForce.SetAll(0.0);
    debug.SetAll(0.0);
    StateTable.AddData(ControllerTime, "ControllerTime");
    StateTable.AddData(ControllerExecTime, "ControllerExecTime");
    StateTable.AddData(JointPos, "PositionJoint");
    StateTable.AddData(JointPosParam, "PositionJointParam");
    StateTable.AddData(JointTargetPos, "PositionTargetJoint");
    StateTable.AddData(JointVel, "VelocityJoint");
    StateTable.AddData(JointVelParam, "VelocityJointParam");
    StateTable.AddData(JointTargetVel, "VelocityTargetJoint");
    StateTable.AddData(JointState, "JointState");
    StateTable.AddData(CartPos, "PositionCartesian");
    StateTable.AddData(TCPSpeed, "VelocityCartesian");
    StateTable.AddData(CartVelParam, "VelocityCartesianParam");
    StateTable.AddData(TCPForce, "ForceCartesianForce");
    StateTable.AddData(WrenchGet, "ForceCartesianParam");
    StateTable.AddData(debug, "Debug");

    mInterface = AddInterfaceProvided("control");
    if (mInterface) {
        // for Status, Warning and Error with mtsMessage
        mInterface->AddMessageEvents();

        // Standard interfaces (same as dVRK)
        mInterface->AddCommandReadState(this->StateTable, JointPosParam, "GetPositionJoint");
        mInterface->AddCommandReadState(this->StateTable, JointTargetPos, "GetPositionJointDesired");
        mInterface->AddCommandReadState(this->StateTable, JointVelParam, "GetVelocityJoint");
        mInterface->AddCommandReadState(this->StateTable, JointState, "GetStateJoint");
        mInterface->AddCommandReadState(this->StateTable, CartPos, "GetPositionCartesian");
        mInterface->AddCommandReadState(this->StateTable, CartVelParam, "GetVelocityCartesian");
        mInterface->AddCommandReadState(this->StateTable, WrenchGet, "GetWrenchBody");
        mInterface->AddCommandWrite(&mtsUniversalRobotScriptRT::JointVelocityMove, this, "JointVelocityMove");
        mInterface->AddCommandWrite(&mtsUniversalRobotScriptRT::JointPositionMove, this, "JointPositionMove");
        mInterface->AddCommandWrite(&mtsUniversalRobotScriptRT::CartesianPositionMove,this, "CartesianPositionMove");
        mInterface->AddCommandWrite(&mtsUniversalRobotScriptRT::CartesianVelocityMove,this, "CartesianVelocityMove");

        // Following are not yet standardized
        mInterface->AddCommandReadState(StateTable, ControllerTime, "GetControllerTime");
        mInterface->AddCommandReadState(StateTable, ControllerExecTime, "GetControllerExecTime");
        mInterface->AddCommandVoid(&mtsUniversalRobotScriptRT::EnableMotorPower, this, "EnableMotorPower");
        mInterface->AddCommandVoid(&mtsUniversalRobotScriptRT::DisableMotorPower, this, "DisableMotorPower");
        mInterface->AddCommandRead(&mtsUniversalRobotScriptRT::GetConnected, this, "GetConnected");
        mInterface->AddCommandRead(&mtsUniversalRobotScriptRT::GetAveragePeriod, this, "GetAveragePeriod");
        mInterface->AddCommandVoid(&mtsUniversalRobotScriptRT::StopMotion, this, "StopMotion");
        mInterface->AddCommandVoid(&mtsUniversalRobotScriptRT::SetRobotRunningMode, this, "SetRobotRunningMode");
        mInterface->AddCommandVoid(&mtsUniversalRobotScriptRT::SetRobotFreeDriveMode, this, "SetRobotFreeDriveMode");
        mInterface->AddCommandReadState(StateTable, debug, "GetDebug");
        mInterface->AddCommandRead(&mtsUniversalRobotScriptRT::GetVersion, this, "GetVersion");
//        mInterface->AddCommandRead(&mtsUniversalRobotScriptRT::GetPolyscopeVersion, this, "GetPolyscopeVersion");

        mInterface->AddEventVoid(SocketErrorEvent, "SocketError");
        mInterface->AddEventVoid(RobotNotReadyEvent, "RobotNotReady");
        mInterface->AddEventVoid(ReceiveTimeoutEvent, "ReceiveTimeout");
        vctULong2 arg;
        mInterface->AddEventWrite(PacketInvalid, "PacketInvalid", arg);

        // Stats
        mInterface->AddCommandReadState(StateTable, StateTable.PeriodStats,
                                        "GetPeriodStatistics");
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
            CMN_LOG_CLASS_INIT_VERBOSE << "Configure: connected" << std::endl;
        }
        else {
            CMN_LOG_CLASS_INIT_ERROR << "Configure: socket not connected" << std::endl;
        }
    }

//    std::string pver;
//    GetPolyscopeVersion(pver);
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

//    std::string pver;
//    GetPolyscopeVersion(pver);
}

void mtsUniversalRobotScriptRT::Run(void)
{
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
        Sleep(0.008 * cmn_s);
        return;
    }

    // Receive a packet with timeout. We choose a timeout of 500 msec, which is much
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
        for (i = VER_UNKNOWN+1; i < VER_MAX; i++) {
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
                break;
            }
        }
        // If we didn't find a match above, increment the VER_UNKNOWN packet counter
        if (i == VER_MAX)
            PacketCount[VER_UNKNOWN]++;
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
            // should be about 0.008 seconds later than the previous value.
            double timeDiff = base1->time - ControllerTime;
            debug[0] = 1000.0*timeDiff;
            if (timeCheckEnabled && ((timeDiff < 0.004) || (timeDiff > 0.024))) {
                // Could create a different event
                PacketInvalid(vctULong2(numBytes, packageLength));
                // Try to recover next time
                ControllerTime += 0.008;
            }
            else {
                ControllerTime = base1->time;
                JointPos.Assign(base1->qActual);
                JointPosParam.SetPosition(JointPos);
                JointVel.Assign(base1->qdActual);
                JointVelParam.SetVelocity(JointVel);
                JointEffort.Assign(base1->I_Actual);
                JointTargetEffort.Assign(base1->I_Target);

                JointState.Position().Assign(JointPos);
                JointState.Velocity().Assign(JointVel);
                JointState.Effort().Assign(JointEffort);
            }
            module2 *base2 = (version <= VER_18) ? (module2 *)(&((packet_pre_3 *)buffer)->base2)
                                                 : (module2 *)(&((packet_30_31 *)buffer)->base2);
            // Following is documented to be "controller realtime thread execution time"
            // Not sure what this is, or what are the units
            ControllerExecTime = base2->controller_Time;
            debug[1] = ControllerExecTime;

            double *tool_vec = 0;
            if (version < VER_30_31) {
                packet_pre_3 *packet = (packet_pre_3 *)(buffer);
                // Documentation does not specify whether tool_Vector field is
                // the actual or target Cartesian position.
                tool_vec = packet->tool_Vector;
            }
            else if (version >= VER_30_31) {
                packet_30_31 *packet = (packet_30_31 *)(buffer);
                tool_vec = packet->tool_vec_Act;  // actual Cartesian position
            }
            if (tool_vec) {
                vct3 position(tool_vec);
                vct3 orientation(tool_vec+3);
                vctRodriguezRotation3<double> rot(orientation);
                vctDoubleRot3 cartRot(rot);  // rotation matrix, from world frame to the end-effector frame
                vctFrm3 frm(cartRot, position);
                CartPos.SetPosition(frm);
            }
            // Finished with packet; now preserve any extra data for next time
            if (packageLength < static_cast<unsigned long>(numBytes)) {
                memmove(buffer, buffer+packageLength, numBytes-packageLength);
                buffer_idx = numBytes-packageLength;
            }
            else
                buffer_idx = 0;
        }
        else
            buffer_idx = 0;
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
        break;

    case UR_FREE_DRIVE:
        break;

    case UR_POS_MOVING:
        if (JointVel.Norm() == 0.0)
            UR_State = UR_IDLE;
        break;

    case UR_POWERING_ON:
        break;

    case UR_POWERING_OFF:
        break;

    default:
        CMN_LOG_CLASS_RUN_ERROR << "Run: unknown state = " << UR_State << std::endl;
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


bool mtsUniversalRobotScriptRT::SendAndReceive(osaSocket &socket, std::string cmd, std::string &recv)
{
    char buf[100];
    if (socket.Send(cmd) < 0) return false;
    Sleep(0.1);   // wait for reply
    if (socket.Receive(buf, 100) < 0) return false;

    recv = buf;
    return true;
}

void mtsUniversalRobotScriptRT::SetRobotFreeDriveMode(void)
{
    if ((UR_State == UR_IDLE) || (UR_State == UR_FREE_DRIVE)) {
        int ret;
        if (version < VER_30_31) {
            ret = socket.Send("set robotmode freedrive\n");
        } else {
            ret = socket.Send("def saw_ur_freedrive():\n\tfreedrive_mode()\n\tsleep(20)\nend\n");
        }

        if (ret == -1) {
            SocketError();
        } else {
            UR_State = UR_FREE_DRIVE;
            mInterface->SendStatus(this->GetName() + ": set freedrive mode");
        }
    }
    else
        RobotNotReady();   // if not idle, ignore command and raise event
}

void mtsUniversalRobotScriptRT::SetRobotRunningMode(void)
{
    if (UR_State == UR_FREE_DRIVE) {
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
    else
        RobotNotReady();   // if not idle, ignore command and raise event
}

void mtsUniversalRobotScriptRT::EnableMotorPower(void)
{
    // TBD
}

void mtsUniversalRobotScriptRT::DisableMotorPower(void)
{
    //if (socket.Send("powerdown()\n") == -1)
    //    SocketError();
}

void mtsUniversalRobotScriptRT::JointVelocityMove(const prmVelocityJointSet &jtvelSet)
{
    if ((UR_State == UR_IDLE) || (UR_State == UR_VEL_MOVING)) {
        vctDoubleVec jtvel(NB_Actuators);
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
        sprintf(VelCmdString,
                "speedj([%6.4lf, %6.4lf, %6.4lf, %6.4lf, %6.4lf, %6.4lf], %6.4lf, 0.1)\n",
                jtvel[0], jtvel[1], jtvel[2], jtvel[3], jtvel[4], jtvel[5], 1.4);
        strcpy(VelCmdStop, "speedj([0.0, 0.0, 0.0, 0.0, 0.0, 0.0], 1.4, 0.0)\n");
        VelCmdTimeout = 100;   // Number of cycles for command to remain valid
        UR_State = UR_VEL_MOVING;
    }
    else
        RobotNotReady();
}

void mtsUniversalRobotScriptRT::JointPositionMove(const prmPositionJointSet &jtposSet)
{
    char JointPosCmdString[100];
    if (UR_State == UR_IDLE) {
        vctDoubleVec jtpos(NB_Actuators);
        jtposSet.GetGoal(jtpos);
        // For now, we issue a movej command; in the future, we may use a trajectory
        // generator and use servoj.
        sprintf(JointPosCmdString,
                "movej([%6.4lf, %6.4lf, %6.4lf, %6.4lf, %6.4lf, %6.4lf], a=%6.4lf, v=%6.4lf)\n",
                jtpos[0], jtpos[1], jtpos[2], jtpos[3], jtpos[4], jtpos[5], 1.4, 0.2);
        if (socket.Send(JointPosCmdString) == -1)
            SocketError();
        else
            UR_State = UR_POS_MOVING;
    }
    else
        RobotNotReady();
}

void mtsUniversalRobotScriptRT::CartesianVelocityMove(const prmVelocityCartesianSet &CartVel)
{
    if ((UR_State == UR_IDLE) || (UR_State == UR_VEL_MOVING)) {
        vct3 velxyz = CartVel.GetVelocity();
        vct3 velrot = CartVel.GetAngularVelocity();
        // speedl(qd, a, t, aRot)
        sprintf(VelCmdString,
                "speedl([%6.4lf, %6.4lf, %6.4lf, %6.4lf, %6.4lf, %6.4lf], %6.4lf, 0.1)\n",
                velxyz.X(), velxyz.Y(), velxyz.Z(), velrot.X(), velrot.Y(), velrot.Z(), 1.4);
        strcpy(VelCmdStop, "speedl([0.0, 0.0, 0.0, 0.0, 0.0, 0.0], 1.4, 0.0)\n");
        VelCmdTimeout = 100;   // Number of cycles for command to remain valid
        UR_State = UR_VEL_MOVING;
    }
    else
        RobotNotReady();
}

void mtsUniversalRobotScriptRT::CartesianPositionMove(const prmPositionCartesianSet &CartPos)
{
    char CartPosCmdString[100];
    if (UR_State == UR_IDLE) {
        vctDoubleFrm3 cartFrm = CartPos.GetGoal();
        vctRodriguezRotation3<double> rot;
        rot.From(cartFrm.Rotation());  // The rotation vector
        sprintf(CartPosCmdString,
            "movel(p[%6.4lf, %6.4lf, %6.4lf, %6.4lf, %6.4lf, %6.4lf], a=%6.4lf, v=%6.4lf)\n",
            cartFrm.Translation().X(), cartFrm.Translation().Y(), cartFrm.Translation().Z(),
            rot.X(), rot.Y(), rot.Z(), 1.2, 0.08);
        if (socket.Send(CartPosCmdString) == -1)
            SocketError();
        else
            UR_State = UR_POS_MOVING;
    }
    else
        RobotNotReady();
}

void mtsUniversalRobotScriptRT::StopMotion(void)
{
    if (socket.Send("stopj(1.4)\n") == -1)
        SocketError();
}


void mtsUniversalRobotScriptRT::GetPolyscopeVersion(std::string &pver)
{
    osaSocket socketDB;    // Dashboard Server
    if (socketDB.Connect(ipAddress.c_str(), 29999)) {
        std::cout << "Connected to Dashboard Server" << std::endl;
    }
    else {
        CMN_LOG_CLASS_INIT_ERROR << "Socket not connected to dashboard server" << std::endl;
    }

    SendAndReceive(socketDB, "PolyscopeVersion\n", pver);

    std::vector<std::string> strings;
    std::string s;
    std::istringstream f(pver);
    while (getline(f, s, '.')) {
        strings.push_back(s);
    }

    if (strings.size() != 3) {
        std::cerr << "invalid version\n";
    }
    else {
        // FIXME: major in string is 3, however atoi returns 0
        pversion.major = atoi(strings[0].c_str());
        pversion.minor = atoi(strings[1].c_str());
        pversion.bugfix = atoi(strings[2].c_str());
        std::cout << "major = " << strings[0].c_str() << "  minor = " << strings[1] << "  bugfix = " << strings[2] << "\n";
    }

    socketDB.Close();
}
