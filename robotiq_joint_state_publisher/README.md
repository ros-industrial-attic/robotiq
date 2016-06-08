# robotiq_joint_state_publisher

### For Robotiq S-model

Subscribes to Robotiq state messages on "SModelRobotInput" topic, converts the data to joint values, and publishes sensor_msgs/JointState messages on "joint_states" topic for Robotiq S-model.

#### Instructions
Run:

`rosrun robotiq_joint_state_publisher s_model_joint_states`

or

`rosrun robotiq_joint_state_publisher s_model_joint_states _prefix:=<gripper_prefix>`

