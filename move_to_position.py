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

rospy.init_node('ik_mover',
                anonymous=True)

rospy.Subscriber('to_move', geometry_msgs.msg.Pose, move, queue_size=1)

rospy.spin()

'''
while True:
    print "======Enter Position======"
    x = input("x: ")
    y = input("y: ")
    z = input("z: ")
    print "======Generating Plan======"
    pose_target = geometry_msgs.msg.Pose()
    pose_target.orientation.w = 1.0
    pose_target.position.x = x
    pose_target.position.y = y
    pose_target.position.z = z
    group.set_pose_target(pose_target)

    print "======Moving Robot======"
    plan1 = group.plan()
    group.go(wait=True)

    '''
