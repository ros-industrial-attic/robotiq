#include "robotiq_c_model_control/c_model_ethercat_client.h"

#include "robotiq_ethercat/ethercat_manager.h"

// See Robotiq's documentation for the register mapping

// An effort to keep the lines less than 100 char long
namespace robotiq_c_model_control
{

CModelEtherCatClient::CModelEtherCatClient(robotiq_ethercat::EtherCatManager& manager, 
                                                    int slave_no)
  : manager_(manager)
  , slave_no_(slave_no)
{}

/*
  See support.robotiq.com -> manual for the register output meanings
*/
void CModelEtherCatClient::writeOutputs(const GripperOutput& output)
{
  uint8_t map[6] = {0}; // array containing all 6 output registers

  // Pack the Action Request register byte
  map[0] = (output.rACT & 0x1) | ((output.rGTO << 0x3) & 0x8) | ((output.rATR << 0x4) & 0x10);
  // registers 1 & 2 reserved by Robotiq
  map[3] = output.rPR;
  map[4] = output.rSP;
  map[5] = output.rFR;

  for (unsigned i = 0; i < 6; ++i)
  {
    manager_.write(slave_no_, i, map[i]);
  }
}

CModelEtherCatClient::GripperInput CModelEtherCatClient::readInputs() const
{
  uint8_t map[6];

  for (unsigned i = 0; i < 6; ++i)
  {
    map[i] = manager_.readInput(slave_no_, i);
  }

  // Decode Input Registers
  GripperInput input;
  input.gACT = map[0] & 0x1;
  input.gGTO = (map[0] >> 0x3) & 0x1;
  input.gSTA = (map[0] >> 0x4) & 0x3;
  input.gOBJ = (map[0] >> 0x6) & 0x3;
  // map[1] is reserved by the protocol
  input.gFLT = map[2] & 0xF;
  input.gPR = map[3];
  input.gPO = map[4];
  input.gCU = map[5];

  return input;
}

CModelEtherCatClient::GripperOutput CModelEtherCatClient::readOutputs() const
{
  uint8_t map[6];
  for (unsigned i = 0; i < 6; ++i)
  {
    map[i] = manager_.readOutput(slave_no_, i);
  }

  GripperOutput output;
  output.rACT = map[0] & 1;
  output.rGTO = (map[0] >> 0x3) & 0x1;
  output.rATR = (map[0] >> 0x4) & 0x1;
  output.rPR = map[3];
  output.rSP = map[4];
  output.rFR = map[5];

  return output;
}
} // end of robotiq_c_model_control namespace