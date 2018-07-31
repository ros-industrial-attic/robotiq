# robotiq_3f_gripper_joint_state_publisher

### For Robotiq 3F gripper

Subscribes to Robotiq state messages on "Robotiq3FGripperRobotInput" topic, converts the data to joint values, and publishes sensor_msgs/JointState messages on "joint_states" topic for Robotiq 3f-gripper.

#### Instructions
Run:

`rosrun robotiq_3f_gripper_joint_state_publisher robotiq_3f_gripper_joint_states`

or

`rosrun robotiq_3f_gripper_joint_state_publisher robotiq_3f_gripper_joint_states _prefix:=<gripper_prefix>`

