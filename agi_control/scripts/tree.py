#! /usr/bin/env python
# -*- coding: utf-8 -*-

import operator
import rospy
import py_trees
import py_trees.console as console
import py_trees_ros.trees

from behaviors import robot_behaviors
from behaviors import scene_behaviors
from subtasks.get_cube import create_get_cube_subtask
from subtasks.pick import create_pick_subtask
from subtasks.place import create_place_subtask


def agi_ctrl_root():
    """
    This is the root of the behavior tree.
    """
    # Create the root node
    root = py_trees.composites.Sequence(name="AGI Control")

    # Data collection sequence
    data2bb = py_trees.composites.Sequence(name="Data to Blackboard")
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
    ready_pose = robot_behaviors.ReadyPose(name="Ready Pose")

    # Get cube subtask
    get_cube_subtask = create_get_cube_subtask()
    # Pick subtask
    pick_subtask = create_pick_subtask()

    # Place subtask
    place_subtask = create_place_subtask()

    pick_place.add_child(get_cube_subtask)
    pick_place.add_child(pick_subtask)
    pick_place.add_child(place_subtask)
    tasks.add_child(pick_place)
    tasks.add_child(ready_pose)
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
