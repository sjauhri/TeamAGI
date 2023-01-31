#! /usr/bin/env python
# -*- coding: utf-8 -*-

import operator
import rospy
import py_trees
import py_trees.console as console
import py_trees_ros.trees

from behaviors import robot_behaviors
from behaviors import scene_behaviors


def agi_ctrl_root():
    """
    This is the root of the behavior tree.
    """
    # Create the root node
    root = py_trees.composites.Parallel(
        name="AGI Control",
        policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ALL)

    # Data collection
    data2bb = py_trees.composites.Sequence(name="Data to Blackboard")
    data2bb.add_child(scene_behaviors.GetSceneBlocks())

    # Task execution
    tasks = py_trees.composites.Selector(name="Tasks")
    stack_block = py_trees.composites.Sequence(name="Stack Block")
    need_new_block = py_trees.composites.Selector(name="Need New Block")
    get_new_block = py_trees.composites.Sequence(name="Get New Block")
    get_new_block.add_child(scene_behaviors.PopNextBlock())
    get_new_block.add_child(scene_behaviors.GetNextStackLocation())
    pick_block = py_trees.decorators.FailureIsSuccess(
        robot_behaviors.PickBlock(name="Pick Block"))
    place_block = robot_behaviors.PlaceBlock(name="Place Block")

    # Check if the variable next_block is not None
    has_block = py_trees.blackboard.CheckBlackboardVariable(
        name="Has Block",
        variable_name="next_block",
        expected_value='',
        comparison_operator=operator.ne)
    idle = py_trees.behaviours.Running(name="Idle")

    need_new_block.add_child(has_block)
    need_new_block.add_child(get_new_block)
    stack_block.add_child(need_new_block)
    stack_block.add_child(pick_block)
    stack_block.add_child(place_block)
    tasks.add_child(stack_block)
    tasks.add_child(idle)
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
    tree.setup(30000)
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
