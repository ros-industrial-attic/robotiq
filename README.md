# Robotiq

[ROS-Industrial](http://www.ros.org/wiki/Industrial) [robotiq](http://robotiq.com/) [meta-package](http://wiki.ros.org/Metapackages).  See the [ROS wiki](http://ros.org/wiki/robotiq) page for more information.


## Build
This build methods uses:
 - [wstool](http://wiki.ros.org/wstool) for rosinstall.
 - [catkin tools](https://catkin-tools.readthedocs.io/en/latest/) for building.
```
mkdir -p robotiq/src
cd robotiq/src
git clone https://github.com/ros-industrial/robotiq.git
wstool init .
wstool merge robotiq/robotiq.rosinstall
wstool update
catkin build
```

