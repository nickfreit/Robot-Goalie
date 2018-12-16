import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String, Float64MultiArray
from moveit_commander.conversions import pose_to_list

def move(pose_target):
    print "Trying to move to target "
    print pose_target
    group.set_pose_target(pose_target)
    plan = group.plan()
    group.go(wait=True)
    return

sys.argv.append("joint_states:=/robot/joint_states")
moveit_commander.roscpp_initialize(sys.argv)

robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()
group = moveit_commander.MoveGroupCommander("left_arm")

rospy.init_node('ik_movement')
rospy.Subscriber('predicted_position', geometry_msgs.msg.Pose, move, queue_size=1)
rospy.spin()
