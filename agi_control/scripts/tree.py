#! /usr/bin/env python
# -*- coding: utf-8 -*-



import operator
# ROS imports
import rospy

# PyTrees imports
import py_trees
import py_trees.console as console
import py_trees_ros.trees

# AGI imports
from behaviors import robot_behaviors
from behaviors import scene_behaviors
from subtasks.get_cube import create_get_cube_subtask
from subtasks.pick import create_pick_subtask
from subtasks.place import create_place_subtask
from utils.robot_utils import get_gripper_status
from utils.robot_utils import get_arm


def pre_tick_handler(behaviour_tree):
    """
    This function is called before each tick of the behaviour tree.
    """
    #set gripper status in blackboard
    next_cube = py_trees.blackboard.Blackboard().get("next_cube")
    if next_cube is None:
        return
    blackboard = py_trees.blackboard.Blackboard()
    gripper_status = get_gripper_status(get_arm(blackboard.get("next_cube")))
    blackboard.set("gripper_status", gripper_status)


def agi_ctrl_root():
    """
    This is the root of the behavior tree.
    """
    # Create the root node
    root = py_trees.composites.Sequence(name="AGI Control")

    # Data collection sequence
    data2bb = py_trees.composites.Sequence(name="Data to Blackboard")
    # data2bb.add_child(robot_behaviors.ScanTable())
    # Get scene blocks
    data2bb.add_child(scene_behaviors.GetSceneBlocks())

    # Tasks selector
    tasks = py_trees.composites.Sequence(name="Tasks")
    # Stack block sequence
    pick_place = py_trees.composites.Sequence(name="Pick and Place")

    # Place block behavior
    #place_block = robot_behaviors.PlaceBlock(name="Place Block")

    # Idle behavior
    idle = py_trees.behaviours.Running(name="Idle")

    # Ready pose robot
    ready_pose = py_trees.composites.Sequence(name="Ready Pose")
    # ready_pose_guard = py_trees.blackboard.CheckBlackboardVariable(
    #     name="AG - ReadyPose | ReadyPose",
    #     variable_name="status_action_pick",
    #     expected_value=py_trees.common.Status.SUCCESS,
    #     comparison_operator=operator.eq)
    ready_pose_action = robot_behaviors.ReadyPose(name="Ready Pose")

    tasks.add_child(pick_place)
    tasks.add_child(ready_pose)
    # ready_pose.add_child(ready_pose_guard)
    ready_pose.add_child(ready_pose_action)
    root.add_child(data2bb)
    root.add_child(tasks)

    base_tree = py_trees_ros.trees.BehaviourTree(root=root)

    base_tree.setup(150)

    # Get cube subtask
    get_cube_subtree = create_get_cube_subtask()
    get_cube_subtree.setup(150)
    # Pick subtask
    pick_subtree = create_pick_subtask()
    pick_subtree.setup(150)
    # Place subtask
    place_subtree = create_place_subtask()
    place_subtree.setup(150)

    base_tree.insert_subtree(get_cube_subtree, pick_place.id, 0)
    base_tree.insert_subtree(pick_subtree, pick_place.id, 1)
    base_tree.insert_subtree(place_subtree, pick_place.id, 2)

    return base_tree


if __name__ == "__main__":
    rospy.init_node("agi_control")
    console.logdebug("Node initialized")

    tree = agi_ctrl_root()
    console.logdebug("Tree initialized")

    tree.add_pre_tick_handler(pre_tick_handler)
    console.logdebug("Tree created")
    console.logdebug("Tree setup")
    try:
        console.loginfo("Starting AGI Control")
        tree.tick_tock(500)
    except KeyboardInterrupt:
        console.loginfo("KeyboardInterrupt")
        tree.interrupt()
    finally:
        tree.destroy()
        console.loginfo("AGI Control Finished")
