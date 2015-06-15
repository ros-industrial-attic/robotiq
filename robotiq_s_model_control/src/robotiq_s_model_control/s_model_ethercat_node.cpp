#include "ros/ros.h"

#include <boost/bind.hpp>
#include <boost/ref.hpp>

#include "robotiq_s_model_control/s_model_ethercat_client.h"
#include <robotiq_s_model_control/SModel_robot_input.h>
#include "robotiq_ethercat/ethercat_manager.h"


/*
  Note that this code currently works only to control ONE SModel gripper
  attached to ONE network interface. If you want to add more grippers
  to the same network, you'll need to edit the source file.
*/

// Note that you will likely need to run the following on your binary:
// sudo setcap cap_net_raw+ep <filename>


void changeCallback(robotiq_s_model_control::SModelEtherCatClient& client,
                    const robotiq_s_model_control::SModelEtherCatClient::GripperOutput::ConstPtr& msg)
{
  client.writeOutputs(*msg);
}


int main(int argc, char** argv)
{
  using robotiq_ethercat::EtherCatManager;
  using robotiq_s_model_control::SModelEtherCatClient;

  typedef SModelEtherCatClient::GripperOutput GripperOutput;
  typedef SModelEtherCatClient::GripperInput GripperInput;

  ros::init(argc, argv, "robotiq_s_model_gripper_node");
  
  ros::NodeHandle nh ("~");

  // Parameter names
  std::string ifname;
  int slave_no;
  bool activate;  

  nh.param<std::string>("ifname", ifname, "enp9s0");
  nh.param<int>("slave_number", slave_no, 1);
  nh.param<bool>("activate", activate, true);

  // Start ethercat manager
  EtherCatManager manager(ifname);
  // register client 
  SModelEtherCatClient client(manager, slave_no);

  // conditionally activate the gripper
  if (activate)
  {
    // Check to see if resetting is required? Or always reset?
    GripperOutput out;
    out.rACT = 0x1;
    client.writeOutputs(out);
  }

  // Sorry for the indentation, trying to keep it under 100 chars
  ros::Subscriber sub = 
        nh.subscribe<GripperOutput>("output", 1,
                                    boost::bind(changeCallback, boost::ref(client), _1));

  ros::Publisher pub = nh.advertise<GripperInput>("input", 100);

  ros::Rate rate(10); // 10 Hz

  while (ros::ok()) 
  {
    SModelEtherCatClient::GripperInput input = client.readInputs();
    pub.publish(input);
    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}
