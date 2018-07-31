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

#include "robotiq_3f_gripper_control/robotiq_3f_gripper_api.h"

using namespace robotiq_3f_gripper_control;
using namespace robotiq;

Robotiq3FGripperAPI::Robotiq3FGripperAPI(boost::shared_ptr<Robotiq3FGripperClientBase> base)
    :base_(base)
{
    pos_to_ticks_ = 200;
    pos_offset_ = 0;
    sci_to_ticks_ = -532;
    sci_offset_ = 0.22;
    vel_to_ticks_ = 2.94;
    vel_offset_ = 22;
    force_to_ticks_ = 5.7;
    force_offset_ = 15;
    cur_to_ticks_ = 10;

    //! Get current status
    read();
    command_.rACT = status_.gACT;
    command_.rMOD = status_.gMOD;
    command_.rGTO = status_.gGTO;
    command_.rPRA = status_.gPRA;
    command_.rPRB = status_.gPRB;
    command_.rPRC = status_.gPRC;
    command_.rPRS = status_.gPRS;
}

void Robotiq3FGripperAPI::setInitialization(InitializationMode mode)
{
    command_.rACT = (Robotiq3FGripperClientBase::GripperOutput::_rACT_type)mode;
}

void Robotiq3FGripperAPI::setGraspingMode(GraspingMode mode)
{
    command_.rMOD = (Robotiq3FGripperClientBase::GripperOutput::_rMOD_type)mode;
}

void Robotiq3FGripperAPI::setActionMode(ActionMode mode)
{
    command_.rGTO = (Robotiq3FGripperClientBase::GripperOutput::_rGTO_type)mode;
}

void Robotiq3FGripperAPI::setEmergencyRelease(EmergencyRelease release)
{
    command_.rATR = (Robotiq3FGripperClientBase::GripperOutput::_rATR_type)release;
}

void Robotiq3FGripperAPI::setInidividualControlMode(IndividualControl fingers, IndividualControl scissor)
{
    command_.rICF = (Robotiq3FGripperClientBase::GripperOutput::_rICF_type)fingers;
    command_.rICS = (Robotiq3FGripperClientBase::GripperOutput::_rICS_type)scissor;
}

void Robotiq3FGripperAPI::setPosition(const double &posA, const double &posB, const double &posC, const double &posS)
{
    command_.rPRA = limit<Robotiq3FGripperClientBase::GripperOutput::_rPRA_type>(pos_to_ticks_*(posA - pos_offset_));
    command_.rPRB = limit<Robotiq3FGripperClientBase::GripperOutput::_rPRB_type>(pos_to_ticks_*(posB - pos_offset_));
    command_.rPRC = limit<Robotiq3FGripperClientBase::GripperOutput::_rPRC_type>(pos_to_ticks_*(posC - pos_offset_));
    command_.rPRS = limit<Robotiq3FGripperClientBase::GripperOutput::_rPRS_type>(sci_to_ticks_*(posS - sci_offset_));
}

void Robotiq3FGripperAPI::setVelocity(const double &velA, const double &velB, const double &velC, const double &velS)
{
    command_.rSPA = limit<Robotiq3FGripperClientBase::GripperOutput::_rSPA_type>(vel_to_ticks_*(velA - vel_offset_));
    command_.rSPB = limit<Robotiq3FGripperClientBase::GripperOutput::_rSPB_type>(vel_to_ticks_*(velB - vel_offset_));
    command_.rSPC = limit<Robotiq3FGripperClientBase::GripperOutput::_rSPC_type>(vel_to_ticks_*(velC - vel_offset_));
    command_.rSPS = limit<Robotiq3FGripperClientBase::GripperOutput::_rSPS_type>(vel_to_ticks_*(velS - vel_offset_));
}

void Robotiq3FGripperAPI::setForce(const double &fA, const double &fB, const double &fC, const double &fS)
{
    command_.rFRA = limit<Robotiq3FGripperClientBase::GripperOutput::_rFRA_type>(force_to_ticks_*(fA - force_offset_));
    command_.rFRB = limit<Robotiq3FGripperClientBase::GripperOutput::_rFRB_type>(force_to_ticks_*(fB - force_offset_));
    command_.rFRC = limit<Robotiq3FGripperClientBase::GripperOutput::_rFRC_type>(force_to_ticks_*(fC - force_offset_));
    command_.rFRS = limit<Robotiq3FGripperClientBase::GripperOutput::_rFRS_type>(force_to_ticks_*(fS - force_offset_));
}

void Robotiq3FGripperAPI::setRaw(const Robotiq3FGripperClientBase::GripperOutput &raw)
{
    command_ = raw;
}

void Robotiq3FGripperAPI::getPosition(double *posA, double *posB, double *posC, double *posS) const
{
    *posA = (double)status_.gPOA/pos_to_ticks_ + pos_offset_;
    *posB = (double)status_.gPOB/pos_to_ticks_ + pos_offset_;
    *posC = (double)status_.gPOC/pos_to_ticks_ + pos_offset_;
    *posS = (double)status_.gPOS/sci_to_ticks_ + sci_offset_;
}

void Robotiq3FGripperAPI::getPositionCmd(double *posA, double *posB, double *posC, double *posS) const
{
    *posA = (double)status_.gPRA/pos_to_ticks_ + pos_offset_;
    *posB = (double)status_.gPRB/pos_to_ticks_ + pos_offset_;
    *posC = (double)status_.gPRC/pos_to_ticks_ + pos_offset_;
    *posS = (double)status_.gPRS/sci_to_ticks_ + sci_offset_;
}

void Robotiq3FGripperAPI::getCurrent(double *curA, double *curB, double *curC, double *curS) const
{
    *curA = (double)status_.gCUA/cur_to_ticks_;
    *curB = (double)status_.gCUB/cur_to_ticks_;
    *curC = (double)status_.gCUC/cur_to_ticks_;
    *curS = (double)status_.gCUS/cur_to_ticks_;
}

void Robotiq3FGripperAPI::getGripperStatus(InitializationMode *gACT, GraspingMode *gMOD, ActionMode *gGTO, GripperStatus *gIMC, MotionStatus *gSTA) const
{
    *gACT = (InitializationMode)status_.gACT;
    *gMOD = (GraspingMode)status_.gMOD;
    *gGTO = (ActionMode)status_.gGTO;
    *gIMC = (GripperStatus)status_.gIMC;
    *gSTA = (MotionStatus)status_.gSTA;
}

void Robotiq3FGripperAPI::getFaultStatus(FaultStatus *gFLT) const
{
    *gFLT = (FaultStatus)status_.gFLT;
}

void Robotiq3FGripperAPI::getObjectStatus(ObjectStatus *fA, ObjectStatus *fB, ObjectStatus *fC, ObjectStatus *fS) const
{
    *fA = (ObjectStatus)status_.gDTA;
    *fB = (ObjectStatus)status_.gDTB;
    *fC = (ObjectStatus)status_.gDTC;
    *fS = (ObjectStatus)status_.gDTS;
}

void Robotiq3FGripperAPI::getRaw(Robotiq3FGripperClientBase::GripperInput *raw) const
{
    *raw = status_;
}

void Robotiq3FGripperAPI::getCommandPos(double *posA, double *posB, double *posC, double *posS) const
{
    *posA = (double)command_.rPRA/pos_to_ticks_ + pos_offset_;
    *posB = (double)command_.rPRB/pos_to_ticks_ + pos_offset_;
    *posC = (double)command_.rPRC/pos_to_ticks_ + pos_offset_;
    *posS = (double)command_.rPRS/sci_to_ticks_ + sci_offset_;
}

bool Robotiq3FGripperAPI::isInitialized()
{
    return ((InitializationMode)status_.gACT == INIT_ACTIVATION);
}

bool Robotiq3FGripperAPI::isReady()
{
    return ((GripperStatus)status_.gIMC == GRIPPER_READY);
}

bool Robotiq3FGripperAPI::isModeSet(GraspingMode mode)
{
    return ((GraspingMode)status_.gMOD == mode);
}

bool Robotiq3FGripperAPI::isHalted()
{
    return ((ActionMode)status_.gGTO == ACTION_STOP);
}

bool Robotiq3FGripperAPI::isMoving()
{
    return (status_.gSTA == MOTION_STARTED);
}

bool Robotiq3FGripperAPI::isEmergReleaseComplete()
{
    return ((FaultStatus)status_.gFLT == ERROR_AUTOMATIC_RELEASE_COMPLETE);
}

void Robotiq3FGripperAPI::read()
{
    status_ = base_->readInputs();
}

void Robotiq3FGripperAPI::write()
{
    base_->writeOutputs(command_);
}
