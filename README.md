# robot_goalie

## How to run the the baxter goalie program
<ol>
 <li> Connect to the Baxter robot. </li>
 <li> Run start_baxter.sh to enable the robot and start joint_trajectory_action_server. </li>
 <li> Run start_kinect.sh to start the Kinect. </li>
 <li> Run start_moveit.sh to start MoveIt! </li>
 <li> Run move_to_position.py to start up the IK service for Baxter. </li>
 <li> Run filter_kinect.py to start the tracking, restart this every time a new
     ball is thrown. No need to restart anything else. </li>
 <li> Throw the ball and see if Baxter can block it! </li>
</ol>
