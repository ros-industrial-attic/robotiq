#ifndef S_MODEL_CLIENT_BASE_H
#define S_MODEL_CLIENT_BASE_H

#include <ros/ros.h>
#include <robotiq_s_model_control/SModel_robot_output.h>
#include <robotiq_s_model_control/SModel_robot_input.h>

namespace robotiq_s_model_control
{

class SModelClientBase
{
public:
    typedef robotiq_s_model_control::SModel_robot_output GripperOutput;
    typedef robotiq_s_model_control::SModel_robot_input GripperInput;

    virtual void init(ros::NodeHandle nh) {}

    /**
     * \brief Write the given set of control flags to the memory of the gripper
     *
     * @param[in] output The set of output-register values to write to the gripper
     */
    virtual void writeOutputs(const GripperOutput& output) = 0;

    /**
     * \brief Reads set of input-register values from the gripper.
     * \return The gripper input registers as read from the controller IOMap
     */
    virtual GripperInput readInputs() = 0;

    /**
     * \brief Reads set of output-register values from the gripper.
     * \return The gripper output registers as read from the controller IOMap
     */
    virtual GripperOutput readOutputs() = 0;

    virtual ~SModelClientBase() {}

protected:
    SModelClientBase() {}

};

} //end namespace robotiq_s_model_control

#endif // S_MODEL_CLIENT_BASE_H
