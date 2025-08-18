import py_trees_ros
import py_trees
import rospy
from node_config import node
import math

class GetPath(py_trees_ros.actions.ActionClient):

    def initialise(self):
        """
        Get target goal from the blackboard to create an action goal
        """            

        preempted = False
        py_trees.blackboard.Blackboard().set("preempted", False)

        if len(node.detected_posts) > 0: # preemption when post with given pole id is detected
                
            if len(node.poles) == 1 and node.poles[0] in node.detected_posts:
                #rospy.logwarn("Post Detected! Previous goal preempted.")
                py_trees.blackboard.Blackboard().set("preempted", True)
                preempted = True

            elif len(node.poles) == 2:
                node.update_gate()
                if node.gate1_id in node.gate_coordinates or node.gate2_id in node.gate_coordinates:
                    #rospy.logwarn("Gate Detected! Previous goal preempted.")
                    py_trees.blackboard.Blackboard().set("preempted", True)
                    preempted = True

        if not preempted:
        
            if not node.points:
                node.create_points(1, 25, math.pi/4, True)
            
            # set target for the next spiral search point       
            self.action_goal = node.get_goal(node.start_point_x + node.points[node.spiral_points_counter][0],node.start_point_y + node.points[node.spiral_points_counter][1])

            py_trees.blackboard.Blackboard().set("target_goal", self.action_goal)

            node.navigation_status_pub.publish("GOING SPIRAL POINT")

        super(GetPath, self).initialise()

    def update(self):
        """
        On success, set points false -> no points remained, set target_goal -> false to get a new goal, decrement remaining_points
        """

        if node.gps_changed():
            return py_trees.Status.FAILURE

        preempted = py_trees.blackboard.Blackboard().get("preempted")
        if preempted:
            return py_trees.Status.FAILURE

        status = super(GetPath, self).update()

        if node.spiral_points_counter == len(node.points):
            rospy.logwarn("All points visited!")
            py_trees.blackboard.Blackboard().set("remaining_points", False)
            return py_trees.Status.FAILURE

        if status == py_trees.Status.SUCCESS: # if reached

            node.navigation_status_pub.publish("SPIRAL POINT REACHED")
        
            rm = py_trees.blackboard.Blackboard().get("remaining_points")

            # update marker as green
            node.markers.markers[node.spiral_points_counter].color.g = 1
            node.markers.markers[node.spiral_points_counter].color.r = 0
            node.markers.markers[node.spiral_points_counter].color.b = 0
            node.spiral_points_counter += 1

            node.publish_markers()
            rospy.logwarn("Remaining points: %d"%rm)
            py_trees.blackboard.Blackboard().set("remaining_points",rm-1)
            py_trees.blackboard.Blackboard().set("target_goal", False)

        elif status == py_trees.Status.RUNNING: 
            
            if len(node.detected_posts) > 0: # preemption when post with given pole id is detected
                
                if len(node.poles) == 1 and node.poles[0] in node.detected_posts:
                    #rospy.logwarn("Post Detected! Previous goal preempted.")
                    status = py_trees.Status.FAILURE

                elif len(node.poles) == 2:
                    node.update_gate()
                    if node.gate1_id in node.gate_coordinates or node.gate2_id in node.gate_coordinates:
                        #rospy.logwarn("Gate Detected! Previous goal preempted.")
                        status = py_trees.Status.FAILURE

            elif node.markers.markers[node.spiral_points_counter].color.b != 1: # update marker as blue if it is not blue
                node.markers.markers[node.spiral_points_counter].color.b = 1
                node.markers.markers[node.spiral_points_counter].color.r = 0
                node.markers.markers[node.spiral_points_counter].color.g = 0

                node.publish_markers()

        return status