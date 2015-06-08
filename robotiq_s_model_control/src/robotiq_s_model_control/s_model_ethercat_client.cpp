#include "robotiq_s_model_control/s_model_ethercat_client.h"

#include "robotiq_ethercat/ethercat_manager.h"

// See Robotiq's documentation for the register mapping

// An effort to keep the lines less than 100 char long
namespace robotiq_s_model_control
{
SModelEtherCatClient::SModelEtherCatClient(robotiq_ethercat::EtherCatManager& manager, 
                                                    int slave_no)
  : manager_(manager)
  , slave_no_(slave_no)
{}

/*
  See support.robotiq.com -> manual for the register output meanings
*/
void SModelEtherCatClient::writeOutputs(const GripperOutput& output)
{
  uint8_t map[15] = {0}; // array containing all 15 output registers

  // Pack the Action Request register byte
  map[0] = (output.rACT & 0x1) | (output.rMOD << 0x1) & 0x6 | ((output.rGTO << 0x3) & 0x8) | ((output.rATR << 0x4) & 0x10);

  // Pack the Gripper Options register byte
  map[1] =  ((output.rICF << 0x2) & 0x4) | ((output.rICS << 0x3) & 0x8);

  // map[2] is empty

  // Requested Position, Speed and Force (Finger A).
  map[3]  = output.rPRA;
  map[4]  = output.rSPA;
  map[5]  = output.rFRA;

  // Finger B
  map[6]  = output.rPRB;
  map[7]  = output.rSPB;
  map[8]  = output.rFRB;

  // Finger C
  map[9]  = output.rPRC;
  map[10] = output.rSPC;
  map[11] = output.rFRC;

  // Scissor Mode
  map[12] = output.rPRS;
  map[13] = output.rSPS;
  map[14] = output.rFRS;

  for (unsigned i = 0; i < 15; ++i)
  {
    manager_.write(slave_no_, i, map[i]);
  }
}

SModelEtherCatClient::GripperInput SModelEtherCatClient::readInputs() const
{
  uint8_t map[15];

  for (unsigned i = 0; i < 15; ++i)
  {
    map[i] = manager_.readInput(slave_no_, i);
  }

  // Decode Input Registers
  GripperInput input;

  // Gripper Status
  input.gACT = map[0] & 0x1;
  input.gMOD = (map[0] >> 0x1) & 0x3;
  input.gGTO = (map[0] >> 0x3) & 0x1;
  input.gIMC = (map[0] >> 0x4) & 0x3;
  input.gSTA = (map[0] >> 0x6) & 0x3;

  // Object Status
  input.gDTA = map[1] & 0x3;
  input.gDTB = (map[1] >> 0x2) & 0x3;
  input.gDTC = (map[1] >> 0x4) & 0x3;
  input.gDTS = (map[1] >> 0x6) & 0x3;

  // Fault Status
  input.gFLT = map[2] & 0xF;

  // Requested Position, Speed and Force (Finger A).
  input.gPRA = map[3];
  input.gPOA = map[4];
  input.gCUA = map[5];

  // Finger B
  input.gPRB = map[6];
  input.gPOB = map[7];
  input.gCUB = map[8];

  // Finger C
  input.gPRC = map[9];
  input.gPOC = map[10];
  input.gCUC = map[11];

  // Scissor Mode
  input.gPRS = map[12];
  input.gPOS = map[13];
  input.gCUS = map[14];

  return input;
}

SModelEtherCatClient::GripperOutput SModelEtherCatClient::readOutputs() const
{
  uint8_t map[15];
  for (unsigned i = 0; i < 15; ++i)
  {
    map[i] = manager_.readOutput(slave_no_, i);
  }

  GripperOutput output;
  output.rACT = map[0] & 1;
  output.rMOD = (map[0] >> 0x1) & 0x6;
  output.rGTO = (map[0] >> 0x3) & 0x1;
  output.rATR = (map[0] >> 0x4) & 0x1;

  output.rICF = (map[1] >> 0x2) & 0x4;
  output.rICS = (map[1] >> 0x3) & 0x8;

  // Finger A
  output.rPRA = map[3];
  output.rSPA = map[4];
  output.rFRA = map[5];

  // Finger B
  output.rPRB = map[6];
  output.rSPB = map[7];
  output.rFRB = map[8];

  // Finger C
  output.rPRC = map[9];
  output.rSPC = map[10];
  output.rFRC = map[11];

  // Scissor Mode
  output.rPRS = map[12];
  output.rSPS = map[13];
  output.rFRS = map[14];

  return output;
}
} // end of robotiq_s_model_control namespace
