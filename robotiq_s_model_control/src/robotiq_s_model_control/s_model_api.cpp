#include "robotiq_s_model_control/s_model_api.h"

using namespace robotiq_s_model_control;
using namespace robotiq;

SModelAPI::SModelAPI(boost::shared_ptr<SModelClientBase> base)
    :base_(base)
{
    pos_to_ticks_ = 200;
    pos_offset_ = 0;
    sci_to_ticks_ = -1200;
    sci_offset_ = 0.05333;
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

void SModelAPI::setInitialization(InitializationMode mode)
{
    command_.rACT = (u_int8_t)mode;
}

void SModelAPI::setGraspingMode(GraspingMode mode)
{
    command_.rMOD = (u_int8_t)mode;
}

void SModelAPI::setActionMode(ActionMode mode)
{
    command_.rGTO = (u_int8_t)mode;
}

void SModelAPI::setEmergencyRelease(EmergencyRelease release)
{
    command_.rATR = (u_int8_t)release;
}

void SModelAPI::setInidividualControlMode(IndividualControl fingers, IndividualControl scissor)
{
    command_.rICF = (u_int8_t)fingers;
    command_.rICS = (u_int8_t)scissor;
}

void SModelAPI::setPosition(const double &posA, const double &posB, const double &posC, const double &posS)
{
    command_.rPRA = pos_to_ticks_*(posA - pos_offset_);
    command_.rPRB = pos_to_ticks_*(posB - pos_offset_);
    command_.rPRC = pos_to_ticks_*(posC - pos_offset_);
    command_.rPRS = sci_to_ticks_*(posS - sci_offset_);
}

void SModelAPI::setVelocity(const double &velA, const double &velB, const double &velC, const double &velS)
{
    command_.rSPA = vel_to_ticks_*(velA - vel_offset_);
    command_.rSPB = vel_to_ticks_*(velB - vel_offset_);
    command_.rSPC = vel_to_ticks_*(velC - vel_offset_);
    command_.rSPS = vel_to_ticks_*(velS - vel_offset_);
}

void SModelAPI::setForce(const double &fA, const double &fB, const double &fC, const double &fS)
{
    command_.rFRA = force_to_ticks_*(fA - force_offset_);
    command_.rFRB = force_to_ticks_*(fB - force_offset_);
    command_.rFRC = force_to_ticks_*(fC - force_offset_);
    command_.rFRS = force_to_ticks_*(fS - force_offset_);
}

void SModelAPI::setRaw(const SModelClientBase::GripperOutput &raw)
{
    command_ = raw;
}

void SModelAPI::getPosition(double &posA, double &posB, double &posC, double &posS)
{
    posA = (double)status_.gPOA/pos_to_ticks_ + pos_offset_;
    posB = (double)status_.gPOB/pos_to_ticks_ + pos_offset_;
    posC = (double)status_.gPOC/pos_to_ticks_ + pos_offset_;
    posS = (double)status_.gPOS/sci_to_ticks_ + sci_offset_;
}

void SModelAPI::getPositionCmd(double &posA, double &posB, double &posC, double &posS)
{
    posA = (double)status_.gPRA/pos_to_ticks_ + pos_offset_;
    posB = (double)status_.gPRB/pos_to_ticks_ + pos_offset_;
    posC = (double)status_.gPRC/pos_to_ticks_ + pos_offset_;
    posS = (double)status_.gPRS/sci_to_ticks_ + sci_offset_;
}

void SModelAPI::getCurrent(double &curA, double &curB, double &curC, double &curS)
{
    curA = (double)status_.gCUA/cur_to_ticks_;
    curB = (double)status_.gCUB/cur_to_ticks_;
    curC = (double)status_.gCUC/cur_to_ticks_;
    curS = (double)status_.gCUS/cur_to_ticks_;
}

void SModelAPI::getGripperStatus(InitializationMode &gACT, GraspingMode &gMOD, ActionMode &gGTO, GripperStatus &gIMC, MotionStatus &gSTA)
{
    gACT = (InitializationMode)status_.gACT;
    gMOD = (GraspingMode)status_.gMOD;
    gGTO = (ActionMode)status_.gGTO;
    gIMC = (GripperStatus)status_.gIMC;
    gSTA = (MotionStatus)status_.gSTA;
}

void SModelAPI::getFaultStatus(FaultStatus &gFLT)
{
    gFLT = (FaultStatus)status_.gFLT;
}

void SModelAPI::getObjectStatus(ObjectStatus &fA, ObjectStatus &fB, ObjectStatus &fC, ObjectStatus &fS)
{
    fA = (ObjectStatus)status_.gDTA;
    fB = (ObjectStatus)status_.gDTB;
    fC = (ObjectStatus)status_.gDTC;
    fS = (ObjectStatus)status_.gDTS;
}

void SModelAPI::getRaw(SModelClientBase::GripperInput &raw)
{
    raw = status_;
}

bool SModelAPI::isInitialized()
{
    return ((InitializationMode)status_.gACT == INIT_ACTIVATION);
}

bool SModelAPI::isReady()
{
    return ((GripperStatus)status_.gIMC == GRIPPER_READY);
}

bool SModelAPI::isModeSet(GraspingMode mode)
{
    return ((GraspingMode)status_.gMOD == mode);
}

bool SModelAPI::isStopped()
{
    return ((ActionMode)status_.gGTO == ACTION_STOP);
}

bool SModelAPI::isEmergReleaseComplete()
{
    return ((FaultStatus)status_.gFLT == ERROR_AUTOMATIC_RELEASE_COMPLETE);
}

void SModelAPI::read()
{
    status_ = base_->readInputs();
}

void SModelAPI::write()
{
    base_->writeOutputs(command_);
}
