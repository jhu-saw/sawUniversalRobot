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

#include <cisstConfig.h>
#include <cisstVector/vctRodriguezRotation3.h>
#include <cisstMultiTask/mtsInterfaceProvided.h>
#include <cisstOSAbstraction/osaSleep.h>
#include <sawUniversalRobot/mtsUniversalRobotScriptRT.h>

const double MAX_VELOCITY = 12.0*cmnPI_180;
const double MIN_VELOCITY = 0.001*cmnPI_180;
const double MAX_ACCELERATION = 4.0*cmnPI_180;

CMN_IMPLEMENT_SERVICES_DERIVED_ONEARG(mtsUniversalRobotScriptRT, mtsTaskContinuous, mtsTaskContinuousConstructorArg)

unsigned long mtsUniversalRobotScriptRT::PacketLength[VER_MAX] = {
       0,  // VER_UNKNOWN
     764,  // VER_PRE_18
     812,  // VER_18
    1044,  // VER_30_31
    1060   // VER_32
};

mtsUniversalRobotScriptRT::mtsUniversalRobotScriptRT(const std::string &name, unsigned int sizeStateTable, bool newThread) :
    mtsTaskContinuous(name, sizeStateTable, newThread), version(VER_UNKNOWN)
{
    Init();
}

mtsUniversalRobotScriptRT::mtsUniversalRobotScriptRT(const mtsTaskContinuousConstructorArg &arg) :
    mtsTaskContinuous(arg), version(VER_UNKNOWN)
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
    StateTable.AddData(CartPos, "PositionCartesian");
    StateTable.AddData(TCPSpeed, "VelocityCartesian");
    StateTable.AddData(CartVelParam, "VelocityCartesianParam");
    StateTable.AddData(TCPForce, "ForceCartesianForce");
    StateTable.AddData(WrenchGet, "ForceCartesianParam");
    StateTable.AddData(debug, "Debug");
  
    mtsInterfaceProvided *provided = AddInterfaceProvided("control");
    if (provided) {
        // Standard interfaces (same as dVRK)
        provided->AddCommandReadState(this->StateTable, JointPosParam, "GetPositionJoint");
        provided->AddCommandReadState(this->StateTable, JointTargetPos, "GetPositionJointDesired");
        provided->AddCommandReadState(this->StateTable, JointVelParam, "GetVelocityJoint");
        provided->AddCommandReadState(this->StateTable, CartPos, "GetPositionCartesian");
        provided->AddCommandReadState(this->StateTable, CartVelParam, "GetVelocityCartesian");
        provided->AddCommandReadState(this->StateTable, WrenchGet, "GetWrenchBody");
        provided->AddCommandWrite(&mtsUniversalRobotScriptRT::JointVelocityMove, this, "JointVelocityMove");
        provided->AddCommandWrite(&mtsUniversalRobotScriptRT::JointPositionMove, this, "JointPositionMove");
        provided->AddCommandWrite(&mtsUniversalRobotScriptRT::CartesianPositionMove,this, "CartesianPositionMove");
        provided->AddCommandWrite(&mtsUniversalRobotScriptRT::CartesianVelocityMove,this, "CartesianVelocityMove");

        // Following are not yet standardized
        provided->AddCommandReadState(StateTable, ControllerTime, "GetControllerTime");
        provided->AddCommandReadState(StateTable, ControllerExecTime, "GetControllerExecTime");
        provided->AddCommandVoid(&mtsUniversalRobotScriptRT::DisableMotorPower, this, "DisableMotorPower");
        provided->AddCommandRead(&mtsUniversalRobotScriptRT::GetConnected, this, "GetConnected");
        provided->AddCommandRead(&mtsUniversalRobotScriptRT::GetAveragePeriod, this, "GetAveragePeriod");
        provided->AddCommandVoid(&mtsUniversalRobotScriptRT::StopMotion, this, "StopMotion");  
        provided->AddCommandVoid(&mtsUniversalRobotScriptRT::SetRobotRunningMode, this, "SetRobotRunningMode");
        provided->AddCommandVoid(&mtsUniversalRobotScriptRT::SetRobotFreeDriveMode, this, "SetRobotFreeDriveMode");
        provided->AddCommandReadState(StateTable, debug, "GetDebug");
        provided->AddCommandRead(&mtsUniversalRobotScriptRT::GetVersion, this, "GetVersion");

        provided->AddEventVoid(SocketError, "SocketError");
        provided->AddEventVoid(RobotNotReady, "RobotNotReady");
        provided->AddEventVoid(ReceiveTimeout, "ReceiveTimeout");
        vctULong2 arg;
        provided->AddEventWrite(PacketInvalid, "PacketInvalid", arg);
    }
}

void mtsUniversalRobotScriptRT::Configure(const std::string &filename)
{
    if (filename.empty())
        CMN_LOG_CLASS_INIT_ERROR << "Configure method requires IP address" << std::endl;
    else {
        ipAddress = filename;
        currentPort = 30003;
        CMN_LOG_CLASS_INIT_VERBOSE << "Connecting to ip " << ipAddress
                                   << ", port " << currentPort << std::endl;
        std::cout << "Connecting to ip " << ipAddress << ", port " << currentPort << " ..." << std::endl;
        if (socket.Connect(ipAddress.c_str(), currentPort)) {
            std::cout << "Connected" << std::endl;
            UR_State = UR_IDLE;
        }
        else
            CMN_LOG_CLASS_INIT_ERROR << "Socket not connected" << std::endl;
    }
}

void mtsUniversalRobotScriptRT::Startup(void)
{
    if (UR_State != UR_NOT_CONNECTED) {
        char buffer[2048];
        // Flush any existing packets
        while (socket.Receive(buffer, sizeof(buffer)) > 0);
    }
}

void mtsUniversalRobotScriptRT::Run(void)
{
    // This buffer must be large enough for largest packet size.
    // According to documentation, port 30003 packets are up to 1060 bytes (Version 3.2)
    // (On port 30001, have seen packets as large as 1295 bytes).
    char buffer[2048];

    // Turn this on to enable sanity check of time difference.
    //bool timeCheckEnabled = (ControllerTime != 0);
    bool timeCheckEnabled = false;

    if (UR_State == UR_NOT_CONNECTED) {
        // Try to connect here?
        // Call any connected components
        RunEvent();
        ProcessQueuedCommands();
        osaSleep(0.008);
        return;
    }

    // Receive a packet with timeout. We choose a timeout of 100 msec, which is much
    // larger than expected (should get packets every 8 msec). Thus, if we don't get
    // a packet, then we raise the ReceiveTimeout event.
    buffer[0] = buffer[1] = buffer[2] = buffer[3] = 0;
    int numBytes = socket.Receive(buffer, sizeof(buffer), 0.1);
    if (numBytes < 0) {
        SocketError();
        socket.Close();
        UR_State = UR_NOT_CONNECTED;
        return;
    }
    if (numBytes == 0) {
        ReceiveTimeout();
        return;
    }

    unsigned long packageLength;
    // Byteswap package length
    char *p = (char *)(&packageLength);
    p[0] = buffer[3]; p[1] = buffer[2]; p[2] = buffer[1]; p[3] = buffer[0];
    if (packageLength == static_cast<unsigned long>(numBytes)) {
        // Byteswap all the doubles in the package
        for (size_t i = 4; i < packageLength-4; i += 8) {
            for (size_t j = 0; j < 4; j++) {
                char tmp = buffer[i + j];
                buffer[i + j] = buffer[i + 7 - j];
                buffer[i + 7 - j] = tmp;
            }
        }
        if (version == VER_UNKNOWN) {
            for (int i = VER_UNKNOWN+1; i < VER_MAX; i++) {
                if (packageLength == PacketLength[i]) {
                    version = static_cast<FirmwareVersion>(i);
                    break;
                }
            }
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
        }
    }
    else {
        PacketInvalid(vctULong2(numBytes, packageLength));
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
        if (socket.Send("freedrive_mode()\n") == -1)
            SocketError();
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

void mtsUniversalRobotScriptRT::SetRobotFreeDriveMode(void)
{
    std::cout << "Free Drive Mode" << std::endl;
    if (UR_State == UR_IDLE) {
        UR_State = UR_FREE_DRIVE;
    }
    else
        RobotNotReady();   // if not idle, ignore command and raise event
}

void mtsUniversalRobotScriptRT::SetRobotRunningMode(void)
{
    std::cout << "Running Mode" << std::endl;
    if (UR_State == UR_FREE_DRIVE) {
        UR_State = UR_IDLE;
    }
    else
        RobotNotReady();   // if not idle, ignore command and raise event
}

void mtsUniversalRobotScriptRT::DisableMotorPower(void)
{
    std::cout << "Shut down robot" << std::endl;
    if (socket.Send("powerdown()\n") == -1)
        SocketError();
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
                "speedj([%6.4lf, %6.4lf, %6.4lf, %6.4lf, %6.4lf, %6.4lf], %6.4lf, 0.0)\n",
                jtvel[0], jtvel[1], jtvel[2], jtvel[3], jtvel[4], jtvel[5], 1.4);
        strcpy(VelCmdStop, "speedj([0.0, 0.0, 0.0, 0.0, 0.0, 0.0], 1.4, 0.0)\n");
        VelCmdTimeout = 100;   // Number of cycles for command to remain valid
        UR_State = UR_VEL_MOVING;
        CMN_LOG_CLASS_RUN_DEBUG << "JointVelocityMove(" << jtvel << "): started at tick = "
                                << GetTick() << std::endl;
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
        else {
            UR_State = UR_POS_MOVING;
            CMN_LOG_CLASS_RUN_DEBUG << "JointPositionMove(" << jtpos << "): started at tick = "
                << GetTick() << std::endl;
        }
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
                "speedl([%6.4lf, %6.4lf, %6.4lf, %6.4lf, %6.4lf, %6.4lf], %6.4lf, 0.0)\n",
                velxyz.X(), velxyz.Y(), velxyz.Z(), velrot.X(), velrot.Y(), velrot.Z(), 1.4);
        strcpy(VelCmdStop, "speedl([0.0, 0.0, 0.0, 0.0, 0.0, 0.0], 1.4, 0.0)\n");
        VelCmdTimeout = 100;   // Number of cycles for command to remain valid
        UR_State = UR_VEL_MOVING;
        CMN_LOG_CLASS_RUN_DEBUG << "CartesianVelocityMove(" << velxyz << "): started at tick = "
                                << GetTick() << std::endl;
    }
    else
        RobotNotReady();
}

void mtsUniversalRobotScriptRT::CartesianPositionMove(const prmPositionCartesianSet &CartPos)
{
    if (UR_State == UR_IDLE) {
        vctDoubleFrm3 cartFrm = CartPos.GetGoal();
        vctRodriguezRotation3<double> rot;
        rot.From(cartFrm.Rotation());  // The rotation vector
        sprintf(CartPosCmdString,
            "movel(p[%6.4lf, %6.4lf, %6.4lf, %6.4lf, %6.4lf, %6.4lf], a=%6.4lf, v=%6.4lf)\n",
            cartFrm.Translation().X(), cartFrm.Translation().Y(), cartFrm.Translation().Z(), rot.X(), rot.Y(), rot.Z(), 1.2, 0.08);
        if (socket.Send(CartPosCmdString) == -1)
            SocketError();
        else {
            UR_State = UR_POS_MOVING;
            CMN_LOG_CLASS_RUN_DEBUG << "CartesianPositionMove(" << cartFrm.Translation() << rot << "): started at tick = "
                << GetTick() << std::endl;
        }
    }
    else
        RobotNotReady();
}

void mtsUniversalRobotScriptRT::StopMotion(void)
{
    if (socket.Send("stopj(1.4)\n") == -1)
        SocketError();
}
