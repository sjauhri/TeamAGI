#! /usr/bin/env python
# -*- coding: utf-8 -*-

# System imports
import rospy

# PyTrees imports
import py_trees
import py_trees.console as console
import py_trees_ros.trees

# Tiago imports
from tiago_dual_pick_place.msg import PlaceAutoObjectAction
from tiago_dual_pick_place.msg import PlaceAutoObjectGoal
from tiago_dual_pick_place.msg import PickUpObjectAction
from tiago_dual_pick_place.msg import PickUpObjectGoal

# ROS imports
from actionlib import SimpleActionClient
from control_msgs.msg import FollowJointTrajectoryAction
from control_msgs.msg import FollowJointTrajectoryGoal
from play_motion_msgs.msg import PlayMotionAction
from play_motion_msgs.msg import PlayMotionGoal
from trajectory_msgs.msg import JointTrajectoryPoint

# AGI imports
from utils.robot_utils import open_gripper

# MoveIt imports
from moveit_commander import PlanningSceneInterface


class PlaceBlock(py_trees_ros.actions.ActionClient):
    """Action Client for placing a block
    
    This class is a wrapper around the ActionClient class from py_trees_ros.
    It generates the goal for the action client and sets the action spec.
    
    Args:
        py_trees_ros.actions.ActionClient ([type]): [description]

    """

    def __init__(self, name="Place Block"):
        super(PlaceBlock, self).__init__(name=name,
                                         action_spec=PlaceAutoObjectAction,
                                         action_namespace="/place_object")

    def initialise(self):
        self.action_goal = self.get_place_goal()
        super(PlaceBlock, self).initialise()

    def get_place_goal(self):
        """Generates the goal for the action client
        
        Returns:
            PlaceAutoObjectGoal: The goal for the action client
        """
        console.loginfo("Getting place goal")
        blackboard = py_trees.blackboard.Blackboard()
        if blackboard.get("next_cube") is None:
            console.logwarn("No more blocks to place")
            return py_trees.common.Status.FAILURE

        place_goal = PlaceAutoObjectGoal()
        place_goal.target_pose = blackboard.get("next_stack_location")
        place_goal.object_name = blackboard.get("next_cube").id
        console.loginfo("Place goal: {}".format(place_goal))
        return place_goal

    def update(self):
        print("Updating place goal")
        blackboard = py_trees.blackboard.Blackboard()
        if blackboard.get("next_cube") is None:
            console.loginfo("No more blocks to place")
            return py_trees.common.Status.FAILURE
        ret = super(PlaceBlock, self).update()
        # If the action succeeded, add the block to the stack and clear the next block variable
        if ret == py_trees.common.Status.SUCCESS:
            blackboard.cubes_in_stack.append(blackboard.next_cube)
            blackboard.set("next_cube", None)
            if self._blackboard.get("cubes_not_in_center") != None:
                blackboard.cubes_not_in_center.append(blackboard.next_cube)
            blackboard.set("next_cube", None)

        console.loginfo("Place goal status: {}".format(ret))
        return ret


class ReadyPose(py_trees.behaviour.Behaviour):
    """Behavior to set the robot in a ready pose
    
    This behavior sets the robot in a ready pose. It is used to wait for the
    next cube to be placed.
    """

    def __init__(self, name="Ready Pose"):
        super(ReadyPose, self).__init__(name=name)

    def setup(self, timeout):
        self.play_m_as = SimpleActionClient('/play_motion', PlayMotionAction)

        self.torso_as = SimpleActionClient(
            '/torso_controller/follow_joint_trajectory',
            FollowJointTrajectoryAction)

        self.head_as = SimpleActionClient(
            '/head_controller/follow_joint_trajectory',
            FollowJointTrajectoryAction)
        super(ReadyPose, self).setup(timeout)

    def initialise(self):
        self.blackboard = py_trees.blackboard.Blackboard()
        self.scene = PlanningSceneInterface()
        console.loginfo("Setting robot in ready pose")

        console.loginfo("Open gripper")
        open_gripper("left")
        open_gripper("right")

        pmg = PlayMotionGoal()
        pmg.motion_name = 'pick_final_pose_l'
        pmg.skip_planning = False
        # self.play_m_as.send_goal(pmg)
        self.play_m_as.send_goal_and_wait(pmg)

        pmg = PlayMotionGoal()
        pmg.motion_name = 'pick_final_pose_r'
        pmg.skip_planning = False
        # self.play_m_as.send_goal(pmg)
        self.play_m_as.send_goal_and_wait(pmg)

        console.loginfo("Lowering torso")
        self.torso_as.wait_for_server()
        goal = FollowJointTrajectoryGoal()
        goal.trajectory.joint_names = ['torso_lift_joint']
        # TODO check height of torso in ready pose!
        goal.trajectory.points = [
            JointTrajectoryPoint(positions=[0.0],
                                 velocities=[0.0],
                                 time_from_start=rospy.Duration(2.0))
        ]
        self.torso_as.send_goal_and_wait(goal)

        # Move head
        self.head_as.wait_for_server()
        goal = FollowJointTrajectoryGoal()
        goal.trajectory.joint_names = ['head_1_joint', 'head_2_joint']
        goal.trajectory.points = [
            JointTrajectoryPoint(positions=[0.0, 0.0],
                                 velocities=[0.0, 0.0],
                                 time_from_start=rospy.Duration(2.0)),
            JointTrajectoryPoint(positions=[0.0, 0.0],
                                 velocities=[0.0, 0.0],
                                 time_from_start=rospy.Duration(4.0))
        ]
        #self.head_as.send_goal(goal)

        console.loginfo("Robot prepared.")

    def update(self):
        return py_trees.common.Status.SUCCESS