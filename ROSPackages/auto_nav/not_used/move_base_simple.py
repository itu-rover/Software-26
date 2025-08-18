#!/usr/bin/env python

# writer: Berat Dalsuna

# some initial tests has been done with this file, tree is not optimized!

##############################################################################
# Imports
##############################################################################

import functools
import py_trees
import py_trees_ros
import py_trees.console as console
import rospy
import sys

import geometry_msgs.msg as geometry_msgs

# imports added by berat
import py_trees.decorators
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_msgs.msg import String

def create_root():
    # Create all behaviours
    bt_root = py_trees.composites.Sequence("Simple Navigation Demo")
    get_goal = py_trees.composites.Sequence("GetGoal")
    new_goal = py_trees_ros.subscribers.ToBlackboard(name="NewGoal",
                                                     topic_name="/locomove_base/current_goal",
                                                     topic_type=geometry_msgs.PoseStamped,
                                                     blackboard_variables = {'target_pose': None})

    is_reached = py_trees_ros.subscribers.ToBlackboard(name="IsReached",
                                                     topic_name="/locomove_base/locomove_base/is_reached",
                                                     topic_type=String,
                                                     blackboard_variables = {'is_reached': None})

    have_goal = py_trees.blackboard.CheckBlackboardVariable(name="Have Goal?", variable_name="target_pose")
    clr_goal = py_trees.blackboard.ClearBlackboardVariable(name="ClearGoal", variable_name="target_pose")
    failure_is_success = py_trees.decorators.Inverter(
        name="Inverter",
        child=have_goal
    )

    bt_root.add_children([get_goal,is_reached,clr_goal])
    get_goal.add_children([failure_is_success,new_goal])

    return bt_root


def shutdown(behaviour_tree):
    behaviour_tree.interrupt()

if __name__ == '__main__':
    rospy.init_node("simple_nav_demo")
    root = create_root()
    behaviour_tree = py_trees_ros.trees.BehaviourTree(root)
    rospy.on_shutdown(functools.partial(shutdown, behaviour_tree))
    if not behaviour_tree.setup(timeout=15):
        console.logerror("failed to setup the tree, aborting.")
        sys.exit(1)

    behaviour_tree.tick_tock(500)

