## Robotiq Ethercat
This package provides an interface for interfacing with an ethercat network. With the current architecture, a single process is associated with a single network and that process must be aware of all slaves it will communicate with.

### Maintainer
- Jean-Philippe Roberge (ros@robotiq.com)

## Common Issues
Please note:
 - VMs probably won't work.
 - You need a dedicated ethernet card for the EtherCAT network. I haven't had luck using eth0.
 - If you're running Ubuntu, you'll probably need to run ```sudo setcap cap_net_raw+ep``` on the ```robotiq_2f_gripper_ethercat_node``` executable before it will work. Otherwise you'll get a "Could not initialize SOEM" error. This executable can be found under the same package name in your <workspace_home>/devel/ directory.
