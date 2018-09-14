// Copyright (c) 2016, Toyota Research Institute. All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:

// 1. Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.

// 2. Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in the
//    documentation and/or other materials provided with the distribution.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#ifndef ROBOTIQ_3F_GRIPPER_API_H
#define ROBOTIQ_3F_GRIPPER_API_H

#include <robotiq_3f_gripper_control/robotiq_3f_gripper_client_base.h>

namespace robotiq
{
enum InitializationMode { INIT_RESET, INIT_ACTIVATION };
enum GraspingMode { GRASP_BASIC, GRASP_PINCH, GRASP_WIDE, GRASP_SCISSOR };
enum ActionMode { ACTION_STOP, ACTION_GO };
enum GripperStatus { GRIPPER_RESET, GRIPPER_ACTIVATING, GRIPPER_MODE_CHANGE, GRIPPER_READY };
enum MotionStatus { MOTION_STARTED, MOTION_PARTIAL_STOP, MOTION_ALL_STOP, MOTION_COMPLETE };
enum ObjectStatus { OBJECT_MOTION, OBJECT_OPEN_CONTACT, OBJECT_CLOSE_CONTACT, OBJECT_MOTION_COMPLETE };
enum FaultStatus { FAULT_NONE, FAULT_UNKNOWN_1, FAULT_UNKNOWN_2, FAULT_UNKNOWN_3,
                   NOTICE, NOTICE_ACTIVATION_DELAYED, NOTICE_MODE_DELAYED, NOTICE_ACTIVATION_NEEDED,
                   WARNING, WARNING_COMM_NOT_READY, WARNING_MODE, WARNING_AUTOMATIC_RELEASE,
                   ERROR, ERROR_ACTIVATION_FAULT, ERROR_MODE_FAULT, ERROR_AUTOMATIC_RELEASE_COMPLETE };

enum EmergencyRelease { EMERGENCY_RELEASE_IDLE, EMERGENCY_RELEASE_ENGAGED };
enum IndividualControl { IND_CONTROL_OFF, IND_CONTROL_ON };
} // end namespace robotiq

namespace robotiq_3f_gripper_control
{
using namespace robotiq;

class Robotiq3FGripperAPI
{
public:
    Robotiq3FGripperAPI(boost::shared_ptr<Robotiq3FGripperClientBase> base);

    void setInitialization(InitializationMode mode);
    void setGraspingMode(GraspingMode mode);
    void setActionMode(ActionMode mode);
    void setEmergencyRelease(EmergencyRelease release);
    void setInidividualControlMode(IndividualControl fingers, IndividualControl scissor);
    void setPosition(const double &posA, const double &posB=0, const double &posC=0, const double &posS=0);
    void setVelocity(const double &velA, const double &velB=0, const double &velC=0, const double &velS=0);
    void setForce(const double &fA, const double &fB=0, const double &fC=0, const double &fS=0);
    void setRaw(const Robotiq3FGripperClientBase::GripperOutput &raw);

    void getPosition(double *posA, double *posB, double *posC, double *posS) const;
    void getPositionCmd(double *posA, double *posB, double *posC, double *posS) const;
    void getCurrent(double *curA, double *curB, double *curC, double *curS) const;
    void getGripperStatus(InitializationMode *gACT,  GraspingMode *gMOD, ActionMode *gGTO, GripperStatus *gIMC, MotionStatus *gSTA) const;
    void getFaultStatus(FaultStatus *gFLT) const;
    void getObjectStatus(ObjectStatus *fA, ObjectStatus *fB, ObjectStatus *fC, ObjectStatus *fS) const;
    void getRaw(Robotiq3FGripperClientBase::GripperInput *raw) const;

    void getCommandPos(double *posA, double *posB, double *posC, double *posS) const;

    bool isInitialized();
    bool isReady();
    bool isModeSet(GraspingMode mode);
    bool isHalted();
    bool isMoving();
    bool isEmergReleaseComplete();

    void read();
    void write();

private:
    boost::shared_ptr<Robotiq3FGripperClientBase> base_;

    Robotiq3FGripperClientBase::GripperInput status_;
    Robotiq3FGripperClientBase::GripperOutput command_;

    //! conversions
    double pos_to_ticks_;
    double pos_offset_;
    double sci_to_ticks_;
    double sci_offset_;
    double vel_to_ticks_;
    double vel_offset_;
    double force_to_ticks_;
    double force_offset_;
    double cur_to_ticks_;

};

template <typename T>
inline T limit (double value)
{
    value = value < std::numeric_limits<T>::min() ? std::numeric_limits<T>::min() : value;
    value = value > std::numeric_limits<T>::max() ? std::numeric_limits<T>::max() : value;
    return static_cast<T>(value);
}

} //end namespace robotiq_3f_gripper_control

#endif // ROBOTIQ_3F_GRIPPER_API_H
