import py_trees
from node_config import node, node_config

class OneGateDetected(py_trees.behaviour.Behaviour):

    def update(self):
        
        if node.gps_changed():
            return py_trees.Status.FAILURE
        
        node.update_gate()

        if len(node.gate_coordinates) > 0:

            #node.abort_goal()

            #node.stop(1.)

            first_pole = list(node.gate_coordinates.items())[0]
            node.calculate_circle_points(node_config.CIRCLE_RADIUS, first_pole[1][0], first_pole[1][1])
            node.publish_markers()
            py_trees.blackboard.Blackboard().set("circle_point_goal", node.circle_points[node.shortest_index])

            return py_trees.common.Status.SUCCESS

        return py_trees.common.Status.FAILURE