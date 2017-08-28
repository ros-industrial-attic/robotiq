# universal_robot
## Gazebo + MoveIt!
* Please use released version 1.1.10
* http://wiki.ros.org/ur_gazebo
  * To launch the simulated arm and a controller for it
    * `$ roslaunch ur_gazebo ur10.launch`
  * Initialize MoveIt!
    * `$ roslaunch ur10_moveit_config ur10_moveit_planning_execution.launch sim:=true`
  * To control the simulated arm in RViz
    * `$ roslaunch ur10_moveit_config moveit_rviz.launch config:=true`

#### Move the Arm in Simulation
* Move the arm by directly sending joint commands
  * `./universal_robot/tests/simulate_ur10_by_sending_joint_command_via_action_client.py`
    * Instantiate an *actionlib* client
    * Send joint commands as `control_msgs.msg.FollowJointTrajectoryGoal` to *actionlib* server in namespace `/arm_controller/follow_joint_trajectory`
    * Simulate the motion in *gazebo*
    * View the motion in *rviz*

* Move the arm via MoveIt!
  * `./universal_robot/tests/simulate_ur10_by_sending_joint_command_via_MoveIt.py`
    * Interact with `moveit_commander.MoveGroupCommander` and send requests as *end effector transformation*, *goal pose (joint-space goal)*, or *cartesian path* to robot controller
    * Simulate the motion in *gazebo*
    * View the motion in *rviz*


## Issues
* If you encounter a **PATH_TOLERANCE_VIOLATED** or a **GOAL_TOLERANCE_VIOLATED**, refer to the path and goal constraints which are configured in `./universal_robot/ur_gazebo/controller/arm_controller_ur10.yaml`.

## Version
* This is based on the release version ros-indigo-ur-\*:amd64/trusty **1.1.10**-1trusty-20170806-180022-0800, and incorporated with the changes customized for Robbie & Yuri (cob raw3-6) in Interactive Robotics Group.
    ```
    ros-indigo-ur-description:amd64/trusty 1.1.10-1trusty-20170804-080540-0800 uptodate
    ros-indigo-ur3-moveit-config:amd64/trusty 1.1.10-1trusty-20170806-180022-0800 uptodate
    ros-indigo-ur5-moveit-config:amd64/trusty 1.1.10-1trusty-20170806-180527-0800 uptodate
    ros-indigo-ur10-moveit-config:amd64/trusty 1.1.10-1trusty-20170806-180307-0800 uptodate
    ros-indigo-ur-bringup:amd64/trusty 1.1.10-1trusty-20170804-081147-0800 uptodate
    ros-indigo-ur-description:amd64/trusty 1.1.10-1trusty-20170804-080540-0800 uptodate
    ros-indigo-ur-driver:amd64/trusty 1.1.10-1trusty-20170804-080728-0800 uptodate
    ros-indigo-ur-gazebo:amd64/trusty 1.1.10-1trusty-20170809-105146-0800 uptodate
    ros-indigo-ur-kinematics:amd64/trusty 1.1.10-1trusty-20170806-172844-0800 uptodate
    ros-indigo-ur-modern-driver:amd64/trusty 0.0.3-0trusty-20170804-081849-0800 uptodate
    ros-indigo-ur-msgs:amd64/trusty 1.1.10-1trusty-20170804-080214-0800 uptodate
    ```

  * To mix a release version with customizations:
    * `git clone https://github.com/ros-industrial/universal_robot.git`
    * `git checkout 1.1.10`
    * `git remote -v`
    * `git remote rename origin upstream`
    * `git remote add origin <your forked git repo url>`
      * `git remote add origin https://github.com/Shentheman/universal_robot.git`
    * `git fetch origin`
    * `git checkout -b <your new branch name>`
      * `git checkout -b irg-raw3-6`
    * `git status`
    * `git add -A`
    * `git commit -am <commit messages>`
    * `git push origin master`
