#! /usr/bin/env python
# -*- coding: utf-8 -*-

import operator
import rospy
import sys
import py_trees
import py_trees.console as console
import py_trees_ros.trees

from behaviours import robot_behaviours
from behaviours import scene_behaviours
from behaviours import util_behaviours


def agi_ctrl_root():
    """
    This is the root of the behavior tree.
    """
    blackboard = py_trees.blackboard.Blackboard()
    # Create the root node
    root = py_trees.composites.Parallel(
        name="AGI Control",
        policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ALL)

    # Data collection
    data2bb = py_trees.composites.Sequence(name="Data to Blackboard")
    data2bb.add_child(scene_behaviours.GetSceneBlocks())

    # Task execution
    # Layer 1: Tasks
    tasks = py_trees.composites.Selector(name="Tasks")
    stack_block = py_trees.composites.Sequence(name="Stack Block")
    idle = py_trees.behaviours.Running(name="Idle")
    tasks.add_child(stack_block)
    tasks.add_child(idle)

    # Layer 2: Stack Block
    # Check if the variable next_block is not None
    need_new_block = py_trees.composites.Selector(name="Need New Block")
    pick_block = py_trees.composites.Sequence(name="Pick Block")
    place_block = robot_behaviours.PlaceBlock(name="Place Block")
    stack_block.add_child(need_new_block)
    stack_block.add_child(pick_block)
    stack_block.add_child(place_block)

    # Layer 3: Need New Block
    has_block = py_trees.blackboard.CheckBlackboardVariable(
        name="Has Block",
        variable_name="next_block",
        expected_value='',
        comparison_operator=operator.ne)
    get_new_block = py_trees.composites.Sequence(name="Get New Block")
    need_new_block.add_child(has_block)
    need_new_block.add_child(get_new_block)

    # Layer 3: Pick Block
    select_arm = robot_behaviours.SelectArm(name="Select Arm")
    change_arm = py_trees.composites.Selector(name="Change Arm")
    pick = py_trees.decorators.FailureIsSuccess(
        robot_behaviours.PickBlock(name="Pick Block"))
    pick_block.add_child(select_arm)
    pick_block.add_child(change_arm)
    pick_block.add_child(pick)

    # Layer 4: Get New Block
    pop_next_block = scene_behaviours.PopNextBlock(name="Pop Next Block")
    get_next_stack_location = scene_behaviours.GetNextStackLocation(
        name="Get Next Stack Location")
    get_new_block.add_child(pop_next_block)
    get_new_block.add_child(get_next_stack_location)

    # Layer 4: Change Arm
    has_same_arm = util_behaviours.CompareTwoBlackboardVariables(
        variable1="left_right", variable2="left_right_old")
    move_arm_to_rest = robot_behaviours.MoveArmToRest(name="Move Arm to Rest")
    change_arm.add_child(has_same_arm)
    change_arm.add_child(move_arm_to_rest)

    root.add_child(data2bb)
    root.add_child(tasks)
    return root


if __name__ == "__main__":
    rospy.init_node("agi_control")
    console.logdebug("Node initialized")

    root = agi_ctrl_root()
    console.logdebug("Tree initialized")

    tree = py_trees_ros.trees.BehaviourTree(root=root)
    console.logdebug("Tree created")
    tree.setup(10000)
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
