import py_trees_ros
import py_trees
from node_config import node
import rospy

class GoCirclePoint(py_trees_ros.actions.ActionClient):
    def initialise(self):
        """
        """            
        
        circle_point_goal = py_trees.blackboard.Blackboard().get("circle_point_goal") # get coordinates

        if circle_point_goal:
            if node.circle_point_counter <= len(node.circle_points):
                rospy.loginfo("Going Circle Point #%d"%node.circle_point_counter)
                node.navigation_status_pub.publish("GOING CIRCLE POINT FOR GATE")
                self.action_goal = node.get_goal(circle_point_goal[0],circle_point_goal[1], rotate_to_center=True)

            else:
                rospy.logwarn("Reached all circle points!")
                
        else:
            rospy.logwarn("No circle point goals!")

        super(GoCirclePoint, self).initialise()

    def update(self):
        """
        """

        status = super(GoCirclePoint, self).update()

        if status == py_trees.Status.SUCCESS:
            rospy.loginfo("Circle Point reached! Moving to next point...")    
            node.circle_point_counter += 1

            if node.circle_point_counter == len(node.circle_points) + 1:
                rospy.logwarn("Reached all circle points !!!") 
                gps = py_trees.blackboard.Blackboard().set("gps_xy",False) # mission failed!
                return py_trees.Status.FAILURE   
            
            node.shortest_index = (node.shortest_index + 1) % len(node.circle_points)

            py_trees.blackboard.Blackboard().set("circle_point_goal",node.circle_points[node.shortest_index])
                
            
        elif status == py_trees.Status.RUNNING:
            if node.gps_changed():
                return py_trees.Status.FAILURE
                
            node.update_gate()    
            if len(node.gate_coordinates) == 2:
                return py_trees.Status.SUCCESS

        else:
            rospy.loginfo("Cannot reach point!")

        return status