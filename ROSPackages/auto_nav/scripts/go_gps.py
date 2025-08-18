import py_trees_ros
import py_trees
import rospy
import math
from node_config import node, node_config

# go to gps goal
class GoGPS(py_trees_ros.actions.ActionClient):

    def initialise(self):
    
        """
        Go to given x,y coordinates
        """            

        if node.gps_changed():
            node.delete_all_markers()
            py_trees.blackboard.Blackboard().set("gps_changed", False)
            if not node.stop_flag:
                rospy.logwarn("Going GPS")
                gps = py_trees.blackboard.Blackboard().get("gps_xy")
                node.navigation_status_pub.publish("GOING GPS POINT")
                self.action_goal = node.get_goal(gps[0], gps[1]) # x, y
            else:
                node.stop_flag = False
        #super(GoGPS, self).initialise()

    def update(self):
        """
        On success, continue to search  
        """

        gps = py_trees.blackboard.Blackboard().get("gps_xy")

        if not gps: # no gps coordinates, stop
            return py_trees.Status.FAILURE

        reached = py_trees.blackboard.Blackboard().get("gps_reached")
        if reached: # continue searching if pole(s) given
        
            if node.poles:
                return py_trees.Status.SUCCESS

            else:
                return py_trees.Status.FAILURE

        status = super(GoGPS, self).update()

        if status == py_trees.Status.SUCCESS:
            rospy.logwarn("GPS Point reached!")   
        
            node.flag = True
        
            py_trees.blackboard.Blackboard().set("gps_reached", True)
            node.navigation_status_pub.publish("GPS POINT REACHED")

            if not node.poles: # mission completed
                node.mission_completed()
                return py_trees.Status.FAILURE
            
            else: # start search

                if not node.points:
                    node.create_points(1,25,math.pi/4,True)
                else:
                    node.create_points(1,25,math.pi/4)

                node.publish_markers()
                node.spiral_points_counter = 0
                py_trees.blackboard.Blackboard().set("remaining_points", len(node.points))
                py_trees.blackboard.Blackboard().set("target_goal", False) 
                return py_trees.Status.SUCCESS

        elif status == py_trees.Status.RUNNING:
            if node.gps_changed():
                return py_trees.Status.FAILURE

            elif len(node.poles) == 1 and node.poles[0] in node.detected_posts:
                return py_trees.Status.SUCCESS

            elif len(node.poles) == 2:
                node.update_gate()
                if node.gate1_id in node.gate_coordinates or node.gate2_id in node.gate_coordinates:
                    status = py_trees.Status.SUCCESS
                
        return status