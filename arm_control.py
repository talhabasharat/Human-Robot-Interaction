#!/usr/bin/env python

import rospy
import moveit_commander
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose
from kinova_msgs.msg import SetFingersPositionAction, SetFingersPositionGoal
import actionlib

def main():
    rospy.init_node('kinova_grocery_pick_and_place')

    # Initialize moveit_commander
    moveit_commander.roscpp_initialize(sys.argv)
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    group_name = "arm"
    move_group = moveit_commander.MoveGroupCommander(group_name)

    # Gripper client
    gripper_client = actionlib.SimpleActionClient('/j2n6s300_driver/fingers_action/finger_positions', SetFingersPositionAction)
    gripper_client.wait_for_server()

    # Function to control the gripper
    def set_gripper_position(position):
        goal = SetFingersPositionGoal()
        goal.fingers.finger1 = position
        goal.fingers.finger2 = position
        gripper_client.send_goal(goal)
        gripper_client.wait_for_result(rospy.Duration(5.0))

    # Function to move to a pose
    def move_to_pose(pose):
        move_group.set_pose_target(pose)
        move_group.go(wait=True)
        move_group.stop()
        move_group.clear_pose_targets()

    # Define poses
    pick_pose = Pose()  # Set the appropriate pose for picking up the groceries
    place_pose = Pose() # Set the appropriate pose for placing groceries in the fridge

    # Open gripper
    set_gripper_position(100.0) # Value in percentage, adjust as needed

    # Move to pick pose
    move_to_pose(pick_pose)

    # Close gripper to pick up the bag
    set_gripper_position(0.0) # Adjust as needed for gripping the bag

    # Move to place pose
    move_to_pose(place_pose)

    # Open gripper to release the bag
    set_gripper_position(100.0)

    # Move back to a safe pose
    safe_pose = Pose()  # Define a safe pose
    move_to_pose(safe_pose)

    rospy.signal_shutdown("Task completed")

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
