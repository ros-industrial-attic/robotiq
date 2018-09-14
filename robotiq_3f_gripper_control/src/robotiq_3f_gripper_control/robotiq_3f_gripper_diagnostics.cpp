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

#include "robotiq_3f_gripper_control/robotiq_3f_gripper_diagnostics.h"

using namespace robotiq_3f_gripper_control;

Robotiq3FGripperDiagnostics::Robotiq3FGripperDiagnostics(boost::shared_ptr<robotiq_3f_gripper_control::Robotiq3FGripperAPI> driver, std::string name)
 :driver_(driver)
{
    diagnostics_.setHardwareID("none");
    diagnostics_.add(name, this, &Robotiq3FGripperDiagnostics::getStatus);
}

void Robotiq3FGripperDiagnostics::update()
{
    diagnostics_.update();
}

void Robotiq3FGripperDiagnostics::getStatus(diagnostic_updater::DiagnosticStatusWrapper &stat)
{
    robotiq::FaultStatus fs;
    driver_->getFaultStatus(&fs);

    switch (fs)
    {
    case robotiq::FAULT_NONE:
        stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "No faults. ");
        break;
    case robotiq::NOTICE_ACTIVATION_DELAYED:
        stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "Action delayed, activation (reactivation) must be completed prior to renewed action. ");
        break;
    case robotiq::NOTICE_MODE_DELAYED:
        stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "Action delayed, mode change must  be completed prior t continuing action. ");
        break;
    case robotiq::NOTICE_ACTIVATION_NEEDED:
        stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "The activation bit must be set prior to action. ");
        break;
    case robotiq::WARNING_COMM_NOT_READY:
        stat.summary(diagnostic_msgs::DiagnosticStatus::WARN, "The communication chip is not ready (may be booting). ");
        break;
    case robotiq::WARNING_MODE:
        stat.summary(diagnostic_msgs::DiagnosticStatus::WARN, "Changing mode fault interference detected on scissor (for less than 20 sec). ");
        break;
    case robotiq::WARNING_AUTOMATIC_RELEASE:
        stat.summary(diagnostic_msgs::DiagnosticStatus::WARN, "Automatic release is in progress. ");
        break;
    case robotiq::ERROR_ACTIVATION_FAULT:
        stat.summary(diagnostic_msgs::DiagnosticStatus::ERROR, "Activation fault, verify that no interference or other error occurred. ");
        break;
    case robotiq::ERROR_MODE_FAULT:
        stat.summary(diagnostic_msgs::DiagnosticStatus::ERROR, "Changing mode fault, interference detected on Scissor (for more than 20 sec). ");
        break;
    case robotiq::ERROR_AUTOMATIC_RELEASE_COMPLETE:
        stat.summary(diagnostic_msgs::DiagnosticStatus::ERROR, "Automatic release completed.  Reset and activation is required. ");
        break;
    default:
        stat.summary(diagnostic_msgs::DiagnosticStatus::ERROR, "Unknown fault");
        break;
    }

    robotiq::InitializationMode gACT;
    robotiq::GraspingMode gMOD;
    robotiq::ActionMode gGTO;
    robotiq::GripperStatus gIMC;
    robotiq::MotionStatus gSTA;

    driver_->getGripperStatus(&gACT, &gMOD, &gGTO, &gIMC, &gSTA);

    stat.add("Initialization Status", toString(gACT));
    stat.add("Grasping Operation Mode Status", toString(gMOD));
    stat.add("Action Status", toString(gGTO));
    stat.add("Gripper Status", toString(gIMC));
    stat.add("Motion Status", toString(gSTA));
}

std::string Robotiq3FGripperDiagnostics::toString(robotiq::InitializationMode status)
{
    switch(status)
    {
    case robotiq::INIT_RESET:
        return "Gripper reset. ";
    case robotiq::INIT_ACTIVATION:
        return "Gripper activation. ";
    default:
        return "Unknown. ";
    }
}

std::string Robotiq3FGripperDiagnostics::toString(robotiq::GraspingMode status)
{
    switch(status)
    {
    case robotiq::GRASP_BASIC:
        return "Basic mode. ";
    case robotiq::GRASP_PINCH:
        return "Pinch mode. ";
    case robotiq::GRASP_WIDE:
        return "Wide mode. ";
    case robotiq::GRASP_SCISSOR:
        return "Scissor mode. ";
    default:
        return "Unknown. ";

    }
}

std::string Robotiq3FGripperDiagnostics::toString(robotiq::ActionMode status)
{
    switch(status)
    {
    case robotiq::ACTION_STOP:
        return "Stopped (or performing activation / grasping mode change / automatic release). ";
    case robotiq::ACTION_GO:
        return "Go to Position Request. ";
    default:
        return "Unknown. ";

    }
}

std::string Robotiq3FGripperDiagnostics::toString(robotiq::GripperStatus status)
{
    switch(status)
    {
    case robotiq::GRIPPER_RESET:
        return "Gripper is in reset (or automatic release) state.  See Fault status if Gripper is activated. ";
    case robotiq::GRIPPER_ACTIVATING:
        return "Activation is in progress. ";
    case robotiq::GRIPPER_MODE_CHANGE:
        return "Mode change is in progress. ";
    case robotiq::GRIPPER_READY:
        return "Activation and Mode change are complete. ";
    default:
        return "Unknown. ";
    }
}

std::string Robotiq3FGripperDiagnostics::toString(robotiq::MotionStatus status)
{
    switch(status)
    {
    case robotiq::MOTION_STARTED:
        return "Gripper is in motion towards requested position (only meaningful if ActionMode enabled). ";
    case robotiq::MOTION_PARTIAL_STOP:
        return "Gripper is stopped. One or two fingers stopped before requested position. ";
    case robotiq::MOTION_ALL_STOP:
        return "Gripper is stopped. All fingers stopped before requested position. ";
    case robotiq::MOTION_COMPLETE:
        return "Gripper is stopped.  All fingers reached requested position." ;
    default:
        return "Unknown. ";
    }
}

std::string Robotiq3FGripperDiagnostics::toString(robotiq::FaultStatus status)
{
    switch (status)
    {
    case robotiq::FAULT_NONE:
        return "No fault. ";
    case robotiq::NOTICE_ACTIVATION_DELAYED:
        return "Action delayed, activation (reactivation) must be completed prior to renewed action. ";
    case robotiq::NOTICE_MODE_DELAYED:
        return "Action delayed, mode change must  be completed prior t continuing action. ";
    case robotiq::NOTICE_ACTIVATION_NEEDED:
        return "The activation bit must be set prior to action. ";
    case robotiq::WARNING_COMM_NOT_READY:
        return "The communication chip is not ready (may be booting). ";
    case robotiq::WARNING_MODE:
        return "Changing mode fault interference detected on scissor (for less than 20 sec). ";
    case robotiq::WARNING_AUTOMATIC_RELEASE:
        return "Automatic release is in progress. ";
    case robotiq::ERROR_ACTIVATION_FAULT:
        return "Activation fault, verify that no interference or other error occurred. ";
    case robotiq::ERROR_MODE_FAULT:
        return "Changing mode fault, interference detected on Scissor (for more than 20 sec). ";
    case robotiq::ERROR_AUTOMATIC_RELEASE_COMPLETE:
        return "Automatic release completed.  Reset and activation is required. ";
    default:
        return "Unknown fault. ";
    }
}
