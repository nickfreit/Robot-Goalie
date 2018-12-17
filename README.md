# robot_goalie

## How to run the the Baxter Goalie program

### Connect to the Baxter
To begin using this code, you must connect to the Baxter Research Robot. The instructions for doing so can be found here http://sdk.rethinkrobotics.com/wiki/Workstation_Setup. For this step you will need to install the ROS Kinetic Distribution, and the Baxter SDK tools that are listed on the above link. Once you have this set up, connect to the Baxter by running the `./baxter.sh`. This script should be set-up correctly to connect to Baxter after following the instructions on the above link.

### Run start_baxter.sh to enable Baxter
This file enables the Baxter robot, and starts up the joint_trajectory_action_server, which is necessary to run inverse kinematics command on Baxter. There are no extra dependencies for this step. Leave the terminal containing the joint_trajectory_action_server running.

### Run start_kinect.sh to start Kinect
First, plug the Microsoft Kinect into your computer. In a new terminal, run `./start_kinect.sh` to start up the Kinect service publishing images that the Kinect sees. To run this script, you will need to install the libfreenect library. On a machine running Ubuntu, this can be done by the command `sudo apt-get install ros-kinetic-libfreenect ros-kinetic-freenect-camera ros-kinetic-freenect-launch`. More information on this installation and on using this library can be found here http://sdk.rethinkrobotics.com/wiki/Kinect_basics. 

### Run start_moveit.sh to start MoveIt!
In  new terminal, run `./start_moveit.sh` to start up the MoveIt! inverse kinematics service. This will also attempt to synchronize your computers time to the time of the Baxter robot, so it will ask for a password to ssh into Baxter. By default this password is `rethink`. There are no extra dependencies for this part.

### Run move_to_position.py
In  new terminal, run `python move_to_position.py` to start up the node that listens for predicted positions and attempts to move Baxter's left arm to intercept those positions. This does not have any extra dependencies.

### Run filter_kinect.py
Finally, in one more new terminal, run `python filter_kienct.py`. This should open a window where you are able to see what the Kinect is seeing, and you can try to throw a ball at the Baxter, and if he is able to predict a location, Baxter will attempt to block the ball. This code depends on the OpenCV and numpy libraries that are available for Python. Each time you want to try to throw the ball, restart this program. All of the other programs should be left running uninterrupted.

### Throwing the ball
The maximum distance that the Kinect camera is set to detect is 3.5 meters. Therefore, you should stand farther than 3.5 meters away from the Baxter, and make sure nothing else is moving in between you and Baxter that would result in false detections. The ball should be thrown gently, slowly, and within the line of sight of the Kinect. The Kinect is assumed to be placed, facing forward, on the top of Baxter's head. If the ball is thrown too quickly, it is likely that not enough points will be obtained, and no position will be predicted. 
