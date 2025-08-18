import py_trees
from node_config import node

# check if the points for the spiral search is available
class PointsAvailable(py_trees.behaviour.Behaviour):

    def update(self):

        st = py_trees.blackboard.Blackboard().get("remaining_points")

        if st > 0:
            return py_trees.common.Status.SUCCESS

        else:
            return py_trees.common.Status.FAILURE