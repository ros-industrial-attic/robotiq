#ifndef S_MODEL_API_H
#define S_MODEL_API_H

#include <robotiq_s_model_control/s_model_client_base.h>

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

namespace robotiq_s_model_control
{
using namespace robotiq;

class SModelAPI
{
public:
    SModelAPI(boost::shared_ptr<SModelClientBase> base);

    void setInitialization(InitializationMode mode);
    void setGraspingMode(GraspingMode mode);
    void setActionMode(ActionMode mode);
    void setEmergencyRelease(EmergencyRelease release);
    void setInidividualControlMode(IndividualControl fingers, IndividualControl scissor);
    void setPosition(const double &posA, const double &posB=0, const double &posC=0, const double &posS=0);
    void setVelocity(const double &velA, const double &velB=0, const double &velC=0, const double &velS=0);
    void setForce(const double &fA, const double &fB=0, const double &fC=0, const double &fS=0);
    void setRaw(const SModelClientBase::GripperOutput &raw);

    void getPosition(double &posA, double &posB, double &posC, double &posS);
    void getPositionCmd(double &posA, double &posB, double &posC, double &posS);
    void getCurrent(double &curA, double &curB, double &curC, double &curS);
    void getGripperStatus(InitializationMode &gACT,  GraspingMode &gMOD, ActionMode &gGTO, GripperStatus &gIMC, MotionStatus &gSTA);
    void getFaultStatus(FaultStatus &gFLT);
    void getObjectStatus(ObjectStatus &fA, ObjectStatus &fB, ObjectStatus &fC, ObjectStatus &fS);
    void getRaw(SModelClientBase::GripperInput &raw);

    void getCommandPos(double &posA, double &posB, double &posC, double &posS);

    bool isInitialized();
    bool isReady();
    bool isModeSet(GraspingMode mode);
    bool isHalted();
    bool isMoving();
    bool isEmergReleaseComplete();

    void read();
    void write();

private:
    boost::shared_ptr<SModelClientBase> base_;

    SModelClientBase::GripperInput status_;
    SModelClientBase::GripperOutput command_;

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

} //end namespace robotiq_s_model_control

#endif // S_MODEL_API_H
