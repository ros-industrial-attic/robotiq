#ifndef S_MODEL_CAN_CLIENT_H
#define S_MODEL_CAN_CLIENT_H

#include <robotiq_s_model_control/s_model_client_base.h>
#include <socketcan_interface/socketcan.h>
#include <socketcan_interface/threading.h>

namespace robotiq_s_model_control
{

/**
 * \brief This class provides a client for the EtherCAT manager object that
 *        can translate robot input/output messages and translate them to
 *        the underlying IO Map.
 */

class SModelCanClient : public SModelClientBase
{
public:
    SModelCanClient(unsigned int can_id, boost::shared_ptr<can::DriverInterface> driver);

    virtual ~SModelCanClient();


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

    can::CommInterface::FrameListener::Ptr frame_listener_;
    can::StateInterface::StateListener::Ptr state_listener_;

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

} //end namespace robotiq_s_model_control

#endif // S_MODEL_CAN_CLIENT_H
