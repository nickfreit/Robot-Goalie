# robot_goalie

 1 - Connect to the Baxter robot.
 2 - Run start_baxter.sh to enable the robot and start joint_trajectory_action_server.
 3 - Run start_kinect.sh to start the Kinect.
 4 - Run start_moveit.sh to start MoveIt!
 5 - Run move_to_position.py to start up the IK service for Baxter.
 6 - Run filter_kinect.py to start the tracking, restart this every time a new
     ball is thrown. No need to restart anything else.
 7 - Throw the ball and see if Baxter can block it!
