#include "robotiq_s_model_control/s_model_can_client.h"

using namespace robotiq_s_model_control;

SModelCanClient::SModelCanClient(unsigned int can_id, boost::shared_ptr<can::DriverInterface> driver)
    :can_id_(can_id)
    ,driver_(driver)
{
    resp_header_.id = can_id + 0x580;
}

SModelCanClient::~SModelCanClient()
{
    read_mutex.unlock();
    driver_->shutdown();

}

void SModelCanClient::init(ros::NodeHandle nh)
{
    requestStart();
    //! Set up listeners
    frame_listener_ = driver_->createMsgListener(resp_header_, can::CommInterface::FrameDelegate(this, &SModelCanClient::frameCallback));
    state_listener_ = driver_->createStateListener(can::StateInterface::StateDelegate(this, &SModelCanClient::stateCallback));
}

void SModelCanClient::writeOutputs(const GripperOutput &output)
{
    //! because too much writing is bad with this device, only write when something has changed.
    //! do the check here, or elsewhere?
    //! write mutex?
    if(driver_->getState().isReady())
    {
        can::Frame serial_request;
        serial_request.id = can_id_+0x600;
        serial_request.dlc = (unsigned char)8;
        serial_request.is_error = 0;
        serial_request.is_rtr = 0;
        serial_request.is_extended = 0;
        serial_request.data.assign((unsigned char)0);

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

SModelCanClient::GripperInput SModelCanClient::readInputs() const
{
    if(driver_->getState().isReady())
    {
        can::Frame serial_request;
        serial_request.id = can_id_+0x600;
        serial_request.dlc = (unsigned char)8;
        serial_request.is_error = 0;
        serial_request.is_rtr = 0;
        serial_request.is_extended = 0;
        serial_request.data.assign((unsigned char)0);

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

SModelCanClient::GripperOutput SModelCanClient::readOutputs() const
{
    return output_;
}

void SModelCanClient::stateCallback(const can::State &s)
{
    std::string err;
    driver_->translateError(s.internal_error, err);
    if(!s.internal_error)
    {
        std::cout<<"State: " <<err <<", asio: "<<s.error_code.message()<<std::endl;
        //throw std::runtime_error(s.error_code.message());
    }
    else
    {
        std::cout<<"Error: "<<err<<", asio: "<<s.error_code.message()<<std::endl;
        throw std::runtime_error(err);
    }

}

void SModelCanClient::frameCallback(const can::Frame &f)
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

void SModelCanClient::requestStart()
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

void SModelCanClient::decodeGripperStatus(const u_int8_t &f)
{
    input_.gACT = ((f >> 0) & 0x1);
    input_.gMOD = ((f >> 1) & 0x3);
    input_.gGTO = ((f >> 3) & 0x1);
    input_.gIMC = ((f >> 4) & 0x3);
    input_.gSTA = ((f >> 6) & 0x3);
}

void SModelCanClient::decodeObjectStatus(const u_int8_t &f)
{
    input_.gDTA = ((f >> 0) & 0x3);
    input_.gDTB = ((f >> 2) & 0x3);
    input_.gDTC = ((f >> 4) & 0x3);
    input_.gDTS = ((f >> 6) & 0x3);
}

void SModelCanClient::decodeFaultStatus(const u_int8_t &f)
{
    input_.gFLT = ((f >> 0) & 0xF);
}

void SModelCanClient::decodeFingerAPosCmd(const u_int8_t &f)
{
    input_.gPRA = f;
}

void SModelCanClient::decodeFingerAPos(const u_int8_t &f)
{
    input_.gPOA = f;
}

void SModelCanClient::decodeFingerACurrent(const u_int8_t &f)
{
    input_.gCUA = f;
}

void SModelCanClient::decodeFingerBPosCmd(const u_int8_t &f)
{
    input_.gPRB = f;
}

void SModelCanClient::decodeFingerBPos(const u_int8_t &f)
{
    input_.gPOB = f;
}

void SModelCanClient::decodeFingerBCurrent(const u_int8_t &f)
{
    input_.gCUB = f;
}

void SModelCanClient::decodeFingerCPosCmd(const u_int8_t &f)
{
    input_.gPRC = f;
}

void SModelCanClient::decodeFingerCPos(const u_int8_t &f)
{
    input_.gPOC = f;
}

void SModelCanClient::decodeFingerCCurrent(const u_int8_t &f)
{
    input_.gCUC = f;
}

void SModelCanClient::decodeFingerSPosCmd(const u_int8_t &f)
{
    input_.gPRS = f;
}

void SModelCanClient::decodeFingerSPos(const u_int8_t &f)
{
    input_.gPOS = f;
}

void SModelCanClient::decodeFingerSCurrent(const u_int8_t &f)
{
    input_.gCUS = f;
}

