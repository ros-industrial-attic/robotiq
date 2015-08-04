#include "robotiq_ethercat/ethercat_manager.h"

#include <string.h>

#include <unistd.h>

#include <boost/ref.hpp>
#include <boost/interprocess/sync/scoped_lock.hpp>

#include <soem/ethercattype.h>
#include <soem/nicdrv.h>
#include <soem/ethercatbase.h>
#include <soem/ethercatmain.h>
#include <soem/ethercatdc.h>
#include <soem/ethercatcoe.h>
#include <soem/ethercatfoe.h>
#include <soem/ethercatconfig.h>
#include <soem/ethercatprint.h>

#include <ros/ros.h>

namespace 
{
  static const unsigned THREAD_SLEEP_TIME = 10000; // 10 ms
  static const unsigned EC_TIMEOUTMON = 500;

  void handleErrors()
  {
    /* one ore more slaves are not responding */
    ec_group[0].docheckstate = FALSE;
    ec_readstate();
    for (int slave = 1; slave <= ec_slavecount; slave++)
    {
      if ((ec_slave[slave].group == 0) && (ec_slave[slave].state != EC_STATE_OPERATIONAL))
      {
        ec_group[0].docheckstate = TRUE;
        if (ec_slave[slave].state == (EC_STATE_SAFE_OP + EC_STATE_ERROR))
        {
          ROS_ERROR("ERROR : slave %d is in SAFE_OP + ERROR, attempting ack.\n", slave);
          ec_slave[slave].state = (EC_STATE_SAFE_OP + EC_STATE_ACK);
          ec_writestate(slave);
        }
        else if(ec_slave[slave].state == EC_STATE_SAFE_OP)
        {
          ROS_WARN("WARNING : slave %d is in SAFE_OP, change to OPERATIONAL.\n", slave);
          ec_slave[slave].state = EC_STATE_OPERATIONAL;
          ec_writestate(slave);
        }
        else if(ec_slave[slave].state > 0)
        {
          if (ec_reconfig_slave(slave, EC_TIMEOUTMON))
          {
            ec_slave[slave].islost = FALSE;
            ROS_INFO("MESSAGE : slave %d reconfigured\n",slave);
          }
        }
        else if(!ec_slave[slave].islost)
        {
          /* re-check state */
          ec_statecheck(slave, EC_STATE_OPERATIONAL, EC_TIMEOUTRET);
          if (!ec_slave[slave].state)
          {
            ec_slave[slave].islost = TRUE;
            ROS_ERROR("ERROR : slave %d lost\n",slave);
          }
        }
      }
      if (ec_slave[slave].islost)
      {
        if(!ec_slave[slave].state)
        {
          if (ec_recover_slave(slave, EC_TIMEOUTMON))
          {
            ec_slave[slave].islost = FALSE;
            ROS_INFO("MESSAGE : slave %d recovered\n",slave);
          }
        }
        else
        {
          ec_slave[slave].islost = FALSE;
          ROS_INFO("MESSAGE : slave %d found\n",slave);
        }
      }
    }
  }

  void cycleWorker(boost::mutex& mutex, bool& stop_flag)
  {
    while (!stop_flag) 
    {
      int expected_wkc = (ec_group[0].outputsWKC * 2) + ec_group[0].inputsWKC;
      int sent, wkc;
      {
        boost::mutex::scoped_lock lock(mutex);
        sent = ec_send_processdata();
        wkc = ec_receive_processdata(EC_TIMEOUTRET);
      }

      if (wkc < expected_wkc)
      {
        handleErrors();
      }

      usleep(THREAD_SLEEP_TIME);
    }
  }

} // end of anonymous namespace


robotiq_ethercat::EtherCatManager::EtherCatManager(const std::string& ifname)
  : ifname_(ifname)
  , stop_flag_(false)
{
  if (initSoem(ifname)) 
  {
    cycle_thread_ = boost::thread(cycleWorker, 
                                  boost::ref(iomap_mutex_),
                                  boost::ref(stop_flag_));
  } 
  else 
  {
    // construction failed
    throw EtherCatError("Could not initialize SOEM");
  }
}

robotiq_ethercat::EtherCatManager::~EtherCatManager()
{
  stop_flag_ = true;
  ec_close();
  cycle_thread_.join();
}

void robotiq_ethercat::EtherCatManager::write(int slave_no, uint8_t channel, uint8_t value)
{
  boost::mutex::scoped_lock lock(iomap_mutex_);
  ec_slave[slave_no].outputs[channel] = value;
}

uint8_t robotiq_ethercat::EtherCatManager::readInput(int slave_no, uint8_t channel) const
{
  boost::mutex::scoped_lock lock(iomap_mutex_);
  return ec_slave[slave_no].inputs[channel];
}

uint8_t robotiq_ethercat::EtherCatManager::readOutput(int slave_no, uint8_t channel) const
{
  boost::mutex::scoped_lock lock(iomap_mutex_);
  return ec_slave[slave_no].outputs[channel];
}

bool robotiq_ethercat::EtherCatManager::initSoem(const std::string& ifname)
{
  // Copy string contents because SOEM library doesn't 
    // practice const correctness
    const static unsigned MAX_BUFF_SIZE = 1024;
    char buffer[MAX_BUFF_SIZE];
    size_t name_size = ifname_.size();
    if (name_size > sizeof(buffer) - 1) 
    {
      ROS_ERROR("Ifname %s exceeds maximum size of %u bytes", ifname.c_str(), MAX_BUFF_SIZE);
      return false;
    }
    std::strncpy(buffer, ifname_.c_str(), MAX_BUFF_SIZE);

    ROS_INFO("Initializing etherCAT master");

    if (!ec_init(buffer))
    {
      ROS_ERROR("Could not initialize ethercat driver");
      return false;
    }

    /* find and auto-config slaves */
    if (ec_config_init(FALSE) <= 0)
    {
      ROS_WARN("No slaves found on %s", ifname_.c_str());
      return false;
    }

    ROS_INFO("SOEM found and configured %d slaves", ec_slavecount);

    unsigned map_size = ec_slave[0].Obytes + ec_slave[0].Ibytes;
    iomap_.reset(new uint8_t[map_size]);

    int iomap_size = ec_config_map(iomap_.get());
    ROS_INFO("SOEM IOMap size: %d", iomap_size);

    // locates dc slaves - ???
    ec_configdc();

    // '0' here addresses all slaves
    if (ec_statecheck(0, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE*4) != EC_STATE_SAFE_OP)
    {
      ROS_ERROR("Could not set EC_STATE_SAFE_OP");
      return false;
    }

    /* 
      This section attempts to bring all slaves to operational status. It does so
      by attempting to set the status of all slaves (ec_slave[0]) to operational,
      then proceeding through 40 send/recieve cycles each waiting up to 50 ms for a
      response about the status. 
    */
    ec_slave[0].state = EC_STATE_OPERATIONAL;
    ec_send_processdata();
    ec_receive_processdata(EC_TIMEOUTRET);

    ec_writestate(0);
    int chk = 40;
    do {
      ec_send_processdata();
      ec_receive_processdata(EC_TIMEOUTRET);
      ec_statecheck(0, EC_STATE_OPERATIONAL, 50000); // 50 ms wait for state check
    } while (chk-- && (ec_slave[0].state != EC_STATE_OPERATIONAL));

    if(ec_statecheck(0,EC_STATE_OPERATIONAL, EC_TIMEOUTSTATE) != EC_STATE_OPERATIONAL)
    {
      ROS_ERROR_STREAM("OPERATIONAL state not set, exiting");
      return false;
    }

    ROS_INFO("Finished configuration successfully");
    return true;
}
