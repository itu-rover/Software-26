import py_trees
from node_config import node

# check if a non-visited post is detected
class PostDetected(py_trees.behaviour.Behaviour):

    def update(self):
        
        if len(node.poles) == 2: # go for gate control
            return py_trees.Status.FAILURE
            
        if node.gps_changed():
            return py_trees.Status.FAILURE

        if len(node.detected_posts) > 0:
            
            node.stop(1.)           

            if len(node.detected_posts) > 0:

                for goal in list(node.detected_posts.items()): # list of dict items
                    if goal[0] not in node.gate_coordinates.keys() and goal[0] in node.poles:
                        py_trees.blackboard.Blackboard().set("post_goal",goal)
                        return py_trees.common.Status.SUCCESS

                return py_trees.common.Status.FAILURE
            
        # dictionary: [14: (5,3), 18: (2,1), ...] -> list(items())[0] => post_goal = [14,(5,3)]
        return py_trees.common.Status.FAILURE