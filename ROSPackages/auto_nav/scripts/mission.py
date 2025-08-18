#!/usr/bin/env python3

# writers:
# -> Berat Dalsuna (behavior tree)
# -> Ahmet Mümin Üyüklü (spiral search algorithm)

"""
Spiral Search Demo: Behavior tree implementing a spiral search navigation strategy
for detecting posts.
"""

# All Imports

import functools
import py_trees
import py_trees_ros
import py_trees.console as console
import rospy
import sys
import tf
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import Odometry
from fiducial_msgs.msg import FiducialTransformArray # /fiducial_transforms topic
import math
import tf2_ros
import actionlib
    
from geometry_msgs.msg import Point
from visualization_msgs.msg import MarkerArray, Marker

# node config initialises node variable

from node_config import node, node_config

# Behaviours

from points_available import PointsAvailable
from post_detected import PostDetected 
from one_gate_detected import OneGateDetected
from gate_detected import GateDetected
from all_posts_visited import AllPostsVisited

# Decorators

from decorators import RepeatDecorator, OscillationDetected, GetPathTimeout 

# rotate right left

from rotate_left_right import RotateLeft, RotateRight 

# Actions

from get_path import GetPath
from go_post import GoPost
from go_gate import GoGate
from go_circle_point import GoCirclePoint
from go_gps import GoGPS

# Create Root

def create_root():
    # Create all behaviours
    nav_node = py_trees.composites.Sequence("Autonomous Navigation")


    get_goal = py_trees.composites.Sequence("Get Spiral Goal")
    post_control = py_trees.composites.Sequence("Post Control")
    gate_control = py_trees.composites.Sequence("Gate Control")
    search = py_trees.composites.Selector("Begin Search")

    new_goal = GetPath(name="Go Spiral Point",
                       action_namespace="/move_base",
                       action_spec=MoveBaseAction)

    new_goal_timeout = GetPathTimeout(
        name="Wait 20 sec",
        child=new_goal,
        duration=20.0
    )

    # rotate right 20 times
    repeat_rotate_right = RepeatDecorator(
        name="rotate right 20",
        num_success=20,
        child=RotateRight("Rotate Right")
    )

    # rotate left 40 times
    repeat_rotate_left = RepeatDecorator(
        name="rotate left 40",
        num_success=40,
        child=RotateLeft("Rotate Left")
    )

    rotate_post = py_trees.composites.Sequence("Rotate for Post")
    rotate_post.add_children([repeat_rotate_right, repeat_rotate_left])

    # rotate right 40 times
    repeat_rotate_right_2 = RepeatDecorator(
        name="rotate right 40",
        num_success=20,
        child=RotateRight("Rotate Right")
    )

    # rotate left 80 times 
    repeat_rotate_left_2 = RepeatDecorator(
        name="rotate left 80",
        num_success=40,
        child=RotateLeft("Rotate Left")
    )

    rotate_gate = py_trees.composites.Sequence("Rotate for Gate")
    rotate_gate.add_children([repeat_rotate_right_2, repeat_rotate_left_2])

    py_trees.blackboard.Blackboard().set("remaining_points",len(node.points))

    py_trees.blackboard.Blackboard().set("post_detected",False)

    py_trees.blackboard.Blackboard().set("post_goal",False)

    py_trees.blackboard.Blackboard().set("gps_xy", False)
    py_trees.blackboard.Blackboard().set("gps_changed", False)
    py_trees.blackboard.Blackboard().set("gps_reached", False)

    py_trees.blackboard.Blackboard().set("mission_completed", False)

    check_points = PointsAvailable(name="Points Available?")

    post_detected = PostDetected(name="Post Detected?")

    gate_detected = GateDetected(name="Gate Detected?")

    one_gate_detected = OneGateDetected(name="One Gate Detected?")

    go_post = GoPost(name="Go to Post",
                     action_namespace="/move_base",
                     action_spec=MoveBaseAction)

    go_gate = GoGate(name="Go to Gate",
                     action_namespace="/move_base",
                     action_spec=MoveBaseAction)

    go_gps = GoGPS(name="Go to GPS",
                   action_namespace="/move_base",
                   action_spec=MoveBaseAction)

    repeat_gate = RepeatDecorator(
        name='back,mid,front',
        num_success=3,
        child=go_gate
    )

    go_circle_point = GoCirclePoint(name="Go to Circle Point",
                                    action_namespace="/move_base",
                                    action_spec=MoveBaseAction)

    repeat_circle = RepeatDecorator(
        name="Circle Points",
        num_success=5,
        child=go_circle_point
    )

    find_gate = py_trees.composites.Selector("Find Gate")
    find_gate.add_children([gate_detected,repeat_circle])

    nav_node.add_children([go_gps, search])
    search.add_children([get_goal, post_control, gate_control])
    get_goal.add_children([check_points,new_goal_timeout])
    post_control.add_children([post_detected,rotate_post,go_post])
    gate_control.add_children([one_gate_detected,rotate_gate,find_gate,repeat_gate])

    return nav_node

# main()

def shutdown(behaviour_tree):
    behaviour_tree.interrupt()


if __name__ == '__main__':

    node.stop_pub.publish(False)

    #node.create_points(1,25,math.pi/4,True) # initialized node points
    
    #node.publish_markers()
    
    root = create_root()
    behaviour_tree = py_trees_ros.trees.BehaviourTree(root)
    rospy.on_shutdown(functools.partial(shutdown, behaviour_tree))
    if not behaviour_tree.setup(timeout=15):
        console.logerror("failed to setup the tree, aborting.")
        sys.exit(1)

    behaviour_tree.tick_tock(node_config.TICK_TOCK)

