# Robotiq

[ROS-Industrial][] robotiq meta-package.  See the [ROS wiki][] page for more information.  

## Contents

This repo holds source code for all versions > groovy. For those versions <= groovy see: [SVN repo][]

[ROS-Industrial]: http://www.ros.org/wiki/Industrial
[ROS wiki]: http://ros.org/wiki/robotiq
[SVN repo]: https://code.google.com/p/swri-ros-pkg/source/browse

## Gazebo Simulation of Robotiq Adaptive 3 Finger Gripper

To simulate Robotiq Adaptive 3 Finger (a3f) Gripper in Gazebo Simulator (from grizzly_ur10 and drcsim)

```
roslaunch robotiq_s_model_articulated_gazebo a3f.launch
```

To open and close it (will collide)

```
rostopic pub --once left_hand/command robotiq_s_model_articulated_msgs/SModelRobotOutput {1,2,1,0,0,0,0,255,0,155,0,0,255,0,0,0,0,0}

rostopic pub --once left_hand/command robotiq_s_model_articulated_msgs/SModelRobotOutput {1,1,1,0,0,0,255,255,0,155,0,0,255,0,0,0,0,0}
```
