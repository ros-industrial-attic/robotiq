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

#ifndef ROBOTIQ_3F_GRIPPER_CAN_CLIENT_H
#define ROBOTIQ_3F_GRIPPER_CAN_CLIENT_H

#include <robotiq_3f_gripper_control/robotiq_3f_gripper_client_base.h>
#include <socketcan_interface/socketcan.h>
#include <socketcan_interface/threading.h>

namespace robotiq_3f_gripper_control
{

/**
 * \brief This class provides a client for the EtherCAT manager object that
 *        can translate robot input/output messages and translate them to
 *        the underlying IO Map.
 */

class Robotiq3FGripperCanClient : public Robotiq3FGripperClientBase
{
public:
    Robotiq3FGripperCanClient(unsigned int can_id, boost::shared_ptr<can::DriverInterface> driver);

    virtual ~Robotiq3FGripperCanClient();


    void init(ros::NodeHandle nh);

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
    unsigned int can_id_;
    boost::shared_ptr<can::DriverInterface> driver_;
    std::string can_device_;
    can::Frame::Header resp_header_;

    GripperInput input_;
    GripperOutput output_;

    can::CommInterface::FrameListenerConstSharedPtr frame_listener_;
    can::StateInterface::StateListenerConstSharedPtr state_listener_;

    void frameCallback(const can::Frame &f);
    void stateCallback(const can::State &s);

    void requestStart();

    std::map<unsigned char, unsigned char> prevCmd_;

    mutable boost::timed_mutex read_mutex;

    void decodeGripperStatus(const u_int8_t &f);
    void decodeObjectStatus(const u_int8_t &f);
    void decodeFaultStatus(const u_int8_t &f);
    void decodeFingerAPos(const u_int8_t &f);
    void decodeFingerBPos(const u_int8_t &f);
    void decodeFingerCPos(const u_int8_t &f);
    void decodeFingerSPos(const u_int8_t &f);
    void decodeFingerACurrent(const u_int8_t &f);
    void decodeFingerBCurrent(const u_int8_t &f);
    void decodeFingerCCurrent(const u_int8_t &f);
    void decodeFingerSCurrent(const u_int8_t &f);
    void decodeFingerAPosCmd(const u_int8_t &f);
    void decodeFingerBPosCmd(const u_int8_t &f);
    void decodeFingerCPosCmd(const u_int8_t &f);
    void decodeFingerSPosCmd(const u_int8_t &f);
};

} //end namespace robotiq_3f_gripper_control

#endif // ROBOTIQ_3F_GRIPPER_CAN_CLIENT_H
