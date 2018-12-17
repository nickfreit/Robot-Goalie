#! /bin/bash
sudo date --set="$(ssh ruser@011308P0002.local date)"
roslaunch baxter_moveit_config baxter_grippers.launch left_electric_gripper:=false joint_state:="/robot_joint_state"
