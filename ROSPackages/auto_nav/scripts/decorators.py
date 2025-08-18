import py_trees
import rospy
from node_config import node

class RepeatDecorator(py_trees.decorators.Decorator): # used for back, mid, front
    """
    Taken from https://py-trees.readthedocs.io/en/devel/_modules/py_trees/decorators.html#Repeat
    """

    def __init__(self, name, child, num_success):
        super().__init__(name=name, child=child)
        self.success = 0
        self.num_success = num_success


    def initialise(self):
        """Reset the currently registered number of successes."""
        self.success = 0

    def update(self):
        """
        Repeat until the nth consecutive success.
        """
        
        if node.gps_changed():
            return py_trees.Status.FAILURE

        if self.decorated.status == py_trees.Status.FAILURE:
            self.feedback_message = f"failed, aborting [status: {self.success} success from {self.num_success}]"
            return py_trees.Status.FAILURE
        elif self.decorated.status == py_trees.Status.SUCCESS:
            self.success += 1
            self.feedback_message = (
                f"success [status: {self.success} success from {self.num_success}]"
            )
            if self.success == self.num_success:
                return py_trees.Status.SUCCESS
            else:
                return py_trees.Status.RUNNING
        else:  # RUNNING
            self.feedback_message = (
                f"running [status: {self.success} success from {self.num_success}]"
            )
            return py_trees.Status.RUNNING

# Check if oscillation is detected 
# TODO
class OscillationDetected(py_trees.behaviour.Behaviour):
    
    def update(self):
        #rospy.logwarn("Oscillation not detected!")
        # TODO
        return py_trees.common.Status.SUCCESS  

# Timeout for spiral search
class GetPathTimeout(py_trees.decorators.Timeout):
    
    
    def update(self): # returns failure if timeout

        """
        If rover cannot reach the goal for 20 secs, move on to next point
        """

        status = super(GetPathTimeout, self).update()

        if status == py_trees.Status.FAILURE:
            
            node.update_gate()

            if node.gate_coordinates:
                return status

            if len(node.poles) > 0 and node.poles[0] not in node.detected_posts: # when there is no preemption

                rospy.loginfo("Point cannot reached! Moving to next point...")    
                rm = py_trees.blackboard.Blackboard().get("remaining_points")

                # update marker as red
                node.markers.markers[node.spiral_points_counter].color.g = 0
                node.markers.markers[node.spiral_points_counter].color.r = 1
                node.markers.markers[node.spiral_points_counter].color.b = 0
                node.spiral_points_counter += 1

                node.publish_markers()
                rospy.logwarn("Remaining points: %d"%rm)
                py_trees.blackboard.Blackboard().set("remaining_points",rm-1)
                py_trees.blackboard.Blackboard().set("target_goal", False)     

        return status