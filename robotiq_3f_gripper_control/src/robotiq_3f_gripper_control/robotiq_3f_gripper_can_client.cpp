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

#include "robotiq_3f_gripper_control/robotiq_3f_gripper_can_client.h"

using namespace robotiq_3f_gripper_control;

namespace
{
static unsigned int kResponseOffset = 0x580;
static unsigned int kRequestOffset = 0x600;
}

Robotiq3FGripperCanClient::Robotiq3FGripperCanClient(unsigned int can_id, boost::shared_ptr<can::DriverInterface> driver)
    :can_id_(can_id)
    ,driver_(driver)
{
    resp_header_.id = can_id + kResponseOffset;
}

Robotiq3FGripperCanClient::~Robotiq3FGripperCanClient()
{
    driver_->shutdown();
}

void Robotiq3FGripperCanClient::init(ros::NodeHandle nh)
{
    requestStart();
    //! Set up listeners
    frame_listener_ = driver_->createMsgListener(resp_header_, can::CommInterface::FrameDelegate(this, &Robotiq3FGripperCanClient::frameCallback));
    state_listener_ = driver_->createStateListener(can::StateInterface::StateDelegate(this, &Robotiq3FGripperCanClient::stateCallback));
}

void Robotiq3FGripperCanClient::writeOutputs(const GripperOutput &output)
{
    //! because too much writing is bad with this device, only write when something has changed.
    //! do the check here, or elsewhere?
    //! write mutex?
    if(driver_->getState().isReady())
    {
        can::Frame serial_request;
        serial_request.id = can_id_ + kRequestOffset;
        serial_request.dlc = (unsigned char)8;
        serial_request.is_error = 0;
        serial_request.is_rtr = 0;
        serial_request.is_extended = 0;
        serial_request.data.fill((unsigned char)0);

        serial_request.data[0] = 0x2F;
        serial_request.data[1] = 0x00;
        serial_request.data[2] = 0x22;

        serial_request.data[3] = 0x01; // Gripper status bits
        serial_request.data[4] = 0x00;
        serial_request.data[4] |= (output.rACT & 0x1) << 0;
        serial_request.data[4] |= (output.rMOD & 0x3) << 1;
        serial_request.data[4] |= (output.rGTO & 0x1) << 3;
        serial_request.data[4] |= (output.rATR & 0x1) << 4;
        if(prevCmd_.find(serial_request.data[3]) == prevCmd_.end() || prevCmd_[serial_request.data[3]]!= serial_request.data[4])
        {
            driver_->send(serial_request);
            prevCmd_[serial_request.data[3]] = serial_request.data[4];
        }

        serial_request.data[3] = 0x02; // Individual control bits
        serial_request.data[4] = 0x00;
        serial_request.data[4] |= (output.rICF & 0x1) << 2;
        serial_request.data[4] |= (output.rICS & 0x1) << 3;
        if(prevCmd_.find(serial_request.data[3]) == prevCmd_.end() || prevCmd_[serial_request.data[3]]!= serial_request.data[4])
        {
            driver_->send(serial_request);
            prevCmd_[serial_request.data[3]] = serial_request.data[4];
        }

        unsigned char subindex = 0x04;

        serial_request.data[3] = subindex++; // Finger A position
        serial_request.data[4] = 0x00;
        serial_request.data[4] = output.rPRA;
        if(prevCmd_.find(serial_request.data[3]) == prevCmd_.end() || prevCmd_[serial_request.data[3]]!= serial_request.data[4])
        {
            driver_->send(serial_request);
            prevCmd_[serial_request.data[3]] = serial_request.data[4];
        }

        serial_request.data[3] = subindex++; // Finger A speed
        serial_request.data[4] = 0x00;
        serial_request.data[4] = output.rSPA;
        if(prevCmd_.find(serial_request.data[3]) == prevCmd_.end() || prevCmd_[serial_request.data[3]]!= serial_request.data[4])
        {
            driver_->send(serial_request);
            prevCmd_[serial_request.data[3]] = serial_request.data[4];
        }

        serial_request.data[3] = subindex++; // Finger A force
        serial_request.data[4] = 0x00;
        serial_request.data[4] = output.rFRA;
        if(prevCmd_.find(serial_request.data[3]) == prevCmd_.end() || prevCmd_[serial_request.data[3]]!= serial_request.data[4])
        {
            driver_->send(serial_request);
            prevCmd_[serial_request.data[3]] = serial_request.data[4];
        }

        serial_request.data[3] = subindex++; // Finger B position
        serial_request.data[4] = 0x00;
        serial_request.data[4] = output.rPRB;
        if(prevCmd_.find(serial_request.data[3]) == prevCmd_.end() || prevCmd_[serial_request.data[3]]!= serial_request.data[4])
        {
            driver_->send(serial_request);
            prevCmd_[serial_request.data[3]] = serial_request.data[4];
        }

        serial_request.data[3] = subindex++; // Finger B speed
        serial_request.data[4] = 0x00;
        serial_request.data[4] = output.rSPB;
        if(prevCmd_.find(serial_request.data[3]) == prevCmd_.end() || prevCmd_[serial_request.data[3]]!= serial_request.data[4])
        {
            driver_->send(serial_request);
            prevCmd_[serial_request.data[3]] = serial_request.data[4];
        }

        serial_request.data[3] = subindex++; // Finger B force
        serial_request.data[4] = 0x00;
        serial_request.data[4] = output.rFRB;
        if(prevCmd_.find(serial_request.data[3]) == prevCmd_.end() || prevCmd_[serial_request.data[3]]!= serial_request.data[4])
        {
            driver_->send(serial_request);
            prevCmd_[serial_request.data[3]] = serial_request.data[4];
        }

        serial_request.data[3] = subindex++; // Finger C position
        serial_request.data[4] = 0x00;
        serial_request.data[4] = output.rPRC;
        if(prevCmd_.find(serial_request.data[3]) == prevCmd_.end() || prevCmd_[serial_request.data[3]]!= serial_request.data[4])
        {
            driver_->send(serial_request);
            prevCmd_[serial_request.data[3]] = serial_request.data[4];
        }

        serial_request.data[3] = subindex++; // Finger C speed
        serial_request.data[4] = 0x00;
        serial_request.data[4] = output.rSPC;
        if(prevCmd_.find(serial_request.data[3]) == prevCmd_.end() || prevCmd_[serial_request.data[3]]!= serial_request.data[4])
        {
            driver_->send(serial_request);
            prevCmd_[serial_request.data[3]] = serial_request.data[4];
        }

        serial_request.data[3] = subindex++; // Finger C force
        serial_request.data[4] = 0x00;
        serial_request.data[4] = output.rFRC;
        if(prevCmd_.find(serial_request.data[3]) == prevCmd_.end() || prevCmd_[serial_request.data[3]]!= serial_request.data[4])
        {
            driver_->send(serial_request);
            prevCmd_[serial_request.data[3]] = serial_request.data[4];
        }

        serial_request.data[3] = subindex++; // Finger S position
        serial_request.data[4] = 0x00;
        serial_request.data[4] = output.rPRS;
        if(prevCmd_.find(serial_request.data[3]) == prevCmd_.end() || prevCmd_[serial_request.data[3]]!= serial_request.data[4])
        {
            driver_->send(serial_request);
            prevCmd_[serial_request.data[3]] = serial_request.data[4];
        }

        serial_request.data[3] = subindex++; // Finger S speed
        serial_request.data[4] = 0x00;
        serial_request.data[4] = output.rSPS;
        if(prevCmd_.find(serial_request.data[3]) == prevCmd_.end() || prevCmd_[serial_request.data[3]]!= serial_request.data[4])
        {
            driver_->send(serial_request);
            prevCmd_[serial_request.data[3]] = serial_request.data[4];
        }

        serial_request.data[3] = subindex++; // Finger S force
        serial_request.data[4] = 0x00;
        serial_request.data[4] = output.rFRS;
        if(prevCmd_.find(serial_request.data[3]) == prevCmd_.end() || prevCmd_[serial_request.data[3]]!= serial_request.data[4])
        {
            driver_->send(serial_request);
            prevCmd_[serial_request.data[3]] = serial_request.data[4];
        }

        output_ = output;

    }
}

Robotiq3FGripperCanClient::GripperInput Robotiq3FGripperCanClient::readInputs() const
{
    if(driver_->getState().isReady())
    {
        can::Frame serial_request;
        serial_request.id = can_id_ + kRequestOffset;
        serial_request.dlc = static_cast<unsigned char>(8);
        serial_request.is_error = 0;
        serial_request.is_rtr = 0;
        serial_request.is_extended = 0;
        serial_request.data.fill(static_cast<unsigned char>(0));

        serial_request.data[0] = 0x4F;
        serial_request.data[1] = 0x00;
        serial_request.data[2] = 0x20;

        for(unsigned int i = 1; i < 16; ++i)
        {
            serial_request.data[3] = i;
            if(!read_mutex.timed_lock(boost::posix_time::time_duration(0,0,1,0)))
            {
                std::stringstream err;
                err<<"unable to read response on index "<<i;
                read_mutex.unlock();
                throw std::runtime_error(err.str().c_str());
            }
            driver_->send(serial_request);
        }
        //! try to lock one more time to wait for last read
        if(!read_mutex.timed_lock(boost::posix_time::time_duration(0,0,1,0)))
        {
            std::stringstream err;
            err<<"unable to read response on last index";
            read_mutex.unlock();
            throw std::runtime_error(err.str().c_str());
        }

        read_mutex.unlock();
    }
    return input_;
}

Robotiq3FGripperCanClient::GripperOutput Robotiq3FGripperCanClient::readOutputs() const
{
    return output_;
}

void Robotiq3FGripperCanClient::stateCallback(const can::State &s)
{
    std::string err;
    driver_->translateError(s.internal_error, err);
    if(!s.internal_error)
    {
        ROS_INFO_STREAM("State: " <<err <<", asio: "<<s.error_code.message());
    }
    else
    {
        ROS_ERROR_STREAM("Error: "<<err<<", asio: "<<s.error_code.message());
        throw std::runtime_error(err);
    }

}

void Robotiq3FGripperCanClient::frameCallback(const can::Frame &f)
{
    can::Frame frame = f;

    if(!frame.isValid())
    {
        std::stringstream err;
        err << "Invalid frame from SocketCAN: id " <<std::hex << f.id <<", length "<<(int) f.dlc <<", is_extended: "<<f.is_extended<<", is error: "<<f.is_error<<", is_rtr: "<<f.is_rtr;
        throw std::runtime_error(err.str().c_str());
    }
    if (frame.is_error)
    {
        throw std::runtime_error("Received frame is error");
    }
    if ((int)frame.dlc != 8)
    {
        throw std::runtime_error("Message length must be 8");
    }

    if(f.data[0]==0x60)
    {
        //! received response from write request
    }
    else if (f.data[0]==0x4f && f.data[1]==0x00 && f.data[2] == 0x20)
    {
        switch((int)f.data[3])
        {
        case 0x01:
            decodeGripperStatus(f.data[4]);
            break;
        case 0x02:
            decodeObjectStatus(f.data[4]);
            break;
        case 0x03:
            decodeFaultStatus(f.data[4]);
            break;
        case 0x04:
            decodeFingerAPosCmd(f.data[4]);
            break;
        case 0x05:
            decodeFingerAPos(f.data[4]);
            break;
        case 0x06:
            decodeFingerACurrent(f.data[4]);
            break;
        case 0x07:
            decodeFingerBPosCmd(f.data[4]);
            break;
        case 0x08:
            decodeFingerBPos(f.data[4]);
            break;
        case 0x09:
            decodeFingerBCurrent(f.data[4]);
            break;
        case 0x0A:
            decodeFingerCPosCmd(f.data[4]);
            break;
        case 0x0B:
            decodeFingerCPos(f.data[4]);
            break;
        case 0x0C:
            decodeFingerCCurrent(f.data[4]);
            break;
        case 0x0D:
            decodeFingerSPosCmd(f.data[4]);
            break;
        case 0x0E:
            decodeFingerSPos(f.data[4]);
            break;
        case 0x0F:
            decodeFingerSCurrent(f.data[4]);
            break;
        default:
            //! unknown subindex
            break;

        }
        read_mutex.unlock();
    }
}

void Robotiq3FGripperCanClient::requestStart()
{
    //! initialize node
    can::Frame iFrame;
    iFrame.id = 0x000;
    iFrame.is_error = 0;
    iFrame.is_extended = 0;
    iFrame.is_rtr = 0;
    iFrame.dlc = (unsigned char)2;
    iFrame.data[0] = 0x01;
    iFrame.data[1] = (unsigned char)can_id_;
    driver_->send(iFrame);
}

void Robotiq3FGripperCanClient::decodeGripperStatus(const u_int8_t &f)
{
    input_.gACT = ((f >> 0) & 0x1);
    input_.gMOD = ((f >> 1) & 0x3);
    input_.gGTO = ((f >> 3) & 0x1);
    input_.gIMC = ((f >> 4) & 0x3);
    input_.gSTA = ((f >> 6) & 0x3);
}

void Robotiq3FGripperCanClient::decodeObjectStatus(const u_int8_t &f)
{
    input_.gDTA = ((f >> 0) & 0x3);
    input_.gDTB = ((f >> 2) & 0x3);
    input_.gDTC = ((f >> 4) & 0x3);
    input_.gDTS = ((f >> 6) & 0x3);
}

void Robotiq3FGripperCanClient::decodeFaultStatus(const u_int8_t &f)
{
    input_.gFLT = ((f >> 0) & 0xF);
}

void Robotiq3FGripperCanClient::decodeFingerAPosCmd(const u_int8_t &f)
{
    input_.gPRA = f;
}

void Robotiq3FGripperCanClient::decodeFingerAPos(const u_int8_t &f)
{
    input_.gPOA = f;
}

void Robotiq3FGripperCanClient::decodeFingerACurrent(const u_int8_t &f)
{
    input_.gCUA = f;
}

void Robotiq3FGripperCanClient::decodeFingerBPosCmd(const u_int8_t &f)
{
    input_.gPRB = f;
}

void Robotiq3FGripperCanClient::decodeFingerBPos(const u_int8_t &f)
{
    input_.gPOB = f;
}

void Robotiq3FGripperCanClient::decodeFingerBCurrent(const u_int8_t &f)
{
    input_.gCUB = f;
}

void Robotiq3FGripperCanClient::decodeFingerCPosCmd(const u_int8_t &f)
{
    input_.gPRC = f;
}

void Robotiq3FGripperCanClient::decodeFingerCPos(const u_int8_t &f)
{
    input_.gPOC = f;
}

void Robotiq3FGripperCanClient::decodeFingerCCurrent(const u_int8_t &f)
{
    input_.gCUC = f;
}

void Robotiq3FGripperCanClient::decodeFingerSPosCmd(const u_int8_t &f)
{
    input_.gPRS = f;
}

void Robotiq3FGripperCanClient::decodeFingerSPos(const u_int8_t &f)
{
    input_.gPOS = f;
}

void Robotiq3FGripperCanClient::decodeFingerSCurrent(const u_int8_t &f)
{
    input_.gCUS = f;
}

