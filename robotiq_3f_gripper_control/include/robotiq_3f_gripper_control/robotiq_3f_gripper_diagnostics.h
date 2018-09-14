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

#ifndef ROBOTIQ_3F_GRIPPER_DIAGNOSTICS_H
#define ROBOTIQ_3F_GRIPPER_DIAGNOSTICS_H

#include "robotiq_3f_gripper_control/robotiq_3f_gripper_api.h"
#include <diagnostic_updater/diagnostic_updater.h>

namespace robotiq_3f_gripper_control
{

class Robotiq3FGripperDiagnostics
{
public:
    Robotiq3FGripperDiagnostics(boost::shared_ptr<robotiq_3f_gripper_control::Robotiq3FGripperAPI> driver, std::string name);

    void update();

    void getStatus(diagnostic_updater::DiagnosticStatusWrapper &stat);

protected:
    boost::shared_ptr<robotiq_3f_gripper_control::Robotiq3FGripperAPI> driver_;

    diagnostic_updater::Updater diagnostics_;

    std::string toString(robotiq::InitializationMode status);
    std::string toString(robotiq::GraspingMode status);
    std::string toString(robotiq::ActionMode status);
    std::string toString(robotiq::GripperStatus status);
    std::string toString(robotiq::MotionStatus status);
    std::string toString(robotiq::FaultStatus status);

};
} //end namespace robotiq_3f_gripper_control

#endif // ROBOTIQ_3F_GRIPPER_DIAGNOSTICS_H
