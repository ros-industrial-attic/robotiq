#ifndef S_MODEL_DIAGNOSTICS_H
#define S_MODEL_DIAGNOSTICS_H

#include "robotiq_s_model_control/s_model_api.h"
#include <diagnostic_updater/diagnostic_updater.h>

namespace robotiq_s_model_control
{

class SModelDiagnostics
{
public:
    SModelDiagnostics(boost::shared_ptr<robotiq_s_model_control::SModelAPI> driver, std::string name);

    void update();

    void getStatus(diagnostic_updater::DiagnosticStatusWrapper &stat);

protected:
    boost::shared_ptr<robotiq_s_model_control::SModelAPI> driver_;

    diagnostic_updater::Updater diagnostics_;

    std::string toString(robotiq::InitializationMode status);
    std::string toString(robotiq::GraspingMode status);
    std::string toString(robotiq::ActionMode status);
    std::string toString(robotiq::GripperStatus status);
    std::string toString(robotiq::MotionStatus status);
    std::string toString(robotiq::FaultStatus status);

};
} //end namespace robotiq_s_model_control

#endif // S_MODEL_DIAGNOSTICS_H
