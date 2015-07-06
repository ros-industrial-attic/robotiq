#ifndef ETHERCAT_MANAGER_H
#define ETHERCAT_MANAGER_H

#include <stdexcept>
#include <string>

#include <stdint.h>

#include <boost/scoped_array.hpp>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>

namespace robotiq_ethercat
{

/**
 * \brief EtherCAT exception. Currently this is only thrown in the event
 *        of a failure to construct an EtherCat manager.
 */
class EtherCatError : public std::runtime_error
{
public:
  explicit EtherCatError(const std::string& what)
    : std::runtime_error(what)
  {}
};

/**
 * \brief This class provides a CPP interface to the SimpleOpenEthercatMaster library
 * Given the name of an ethernet device, such as "eth0", it will connect,
 * start a thread that cycles data around the network, and provide read/write
 * access to the underlying io map.
 *
 * Please note that as used in these docs, 'Input' and 'Output' are relative to
 * your program. So the 'Output' registers are the ones you write to, for example.
 */
class EtherCatManager
{
public:
  /**
   * \brief Constructs and initializes the ethercat slaves on a given network interface.
   *
   * @param[in] ifname the name of the network interface that the ethercat chain 
   *                   is connected to (i.e. "eth0")
   *
   * Constructor can throw EtherCatError exception if SOEM could not be 
   * initialized.
   */
  EtherCatManager(const std::string& ifname);
  
  ~EtherCatManager();

  /**
  * \brief writes 'value' to the 'channel-th' output-register of the given 'slave'
  *  
  * @param[in] slave_no The slave number of the device to write to (>= 1)
  * @param[in] channel The byte offset into the output IOMap to write value to
  * @param[in] value The byte value to write
  *
  * This method currently makes no attempt to catch out of bounds errors. Make
  * sure you know your IOMap bounds.
  */
  void write(int slave_no, uint8_t channel, uint8_t value);

  /**
  * \brief Reads the "channel-th" input-register of the given slave no
  *  
  * @param[in] slave_no The slave number of the device to read from (>= 1)
  * @param[in] channel The byte offset into the input IOMap to read from
  */
  uint8_t readInput(int slave_no, uint8_t channel) const;

  /**
  * \brief Reads the "channel-th" output-register of the given slave no
  *  
  * @param[in] slave_no The slave number of the device to read from (>= 1)
  * @param[in] channel The byte offset into the output IOMap to read from
  */
  uint8_t readOutput(int slave_no, uint8_t channel) const;

private:
  bool initSoem(const std::string& ifname);

  const std::string ifname_;
  boost::scoped_array<uint8_t> iomap_;
  boost::thread cycle_thread_;
  mutable boost::mutex iomap_mutex_;
  bool stop_flag_;
};

}

#endif
