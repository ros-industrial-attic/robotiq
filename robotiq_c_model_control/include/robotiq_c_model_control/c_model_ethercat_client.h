#ifndef C_MODEL_ETHERCAT_CLIENT_H
#define C_MODEL_ETHERCAT_CLIENT_H

#include <robotiq_c_model_control/CModel_robot_output.h>
#include <robotiq_c_model_control/CModel_robot_input.h>

// Forward declaration of EtherCatManager
namespace robotiq_ethercat 
{
  class EtherCatManager;
}

namespace robotiq_c_model_control
{

/**
 * \brief This class provides a client for the EtherCAT manager object that
 *        can translate robot input/output messages and translate them to
 *        the underlying IO Map.
 */

class CModelEtherCatClient
{
public:
  typedef robotiq_c_model_control::CModel_robot_output GripperOutput;
  typedef robotiq_c_model_control::CModel_robot_input GripperInput;

  /**
   * \brief Constructs a control interface to a C Model Robotiq gripper on
   *        the given ethercat network and the given slave_no.
   *
   * @param[in] manager The interface to an EtherCAT network that the gripper
   *                    is connected to.
   *
   * @param[in] slave_no The slave number of the gripper on the EtherCAT network
   *                     (>= 1)
   */
  CModelEtherCatClient(robotiq_ethercat::EtherCatManager& manager, int slave_no);

  /**
   * \brief Write the given set of control flags to the memory of the gripper
   * 
   * @param[in] output The set of output-register values to write to the gripper
   */
  void writeOutputs(const GripperOutput& output);
  
  /**
   * \brief Reads set of input-register values from the gripper.
   * \return The gripper input registers as read from the controller IOMap
   */
  GripperInput readInputs() const;
  
  /**
   * \brief Reads set of output-register values from the gripper.
   * \return The gripper output registers as read from the controller IOMap
   */
  GripperOutput readOutputs() const;

private:
  robotiq_ethercat::EtherCatManager& manager_;
  const int slave_no_;
};

}

#endif
