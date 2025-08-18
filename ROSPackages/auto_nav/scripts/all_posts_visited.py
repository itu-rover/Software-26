import py_trees
from node_config import node, node_config

# check if all posts are visited
class AllPostsVisited(py_trees.behaviour.Behaviour):

    def update(self):

        post_count = node.post_counter

        if post_count == node_config.POST_NUMBER:
            rospy.logwarn("All Posts Reached!!!")
            return py_trees.common.Status.SUCCESS

        else:
            return py_trees.common.Status.FAILURE