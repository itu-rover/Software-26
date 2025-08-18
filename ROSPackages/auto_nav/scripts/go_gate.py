import py_trees
import py_trees_ros
import rospy
import math
from node_config import node

# go to detected gate
class GoGate(py_trees_ros.actions.ActionClient):


    def initialise(self):
        """
        Get pos of gate, create 3 goals and pass
        """            

        if node.gps_changed():
            return py_trees.Status.FAILURE  
        
        gate_goals = py_trees.blackboard.Blackboard().get("gate_goals") # get (back,mid,front)
        gate_goal = None

        if gate_goals:
            gate_goal = gate_goals[node.gate_goal_counter]
            print("Gate goal:",gate_goal)
            node.navigation_status_pub.publish(f"GOING GATE POINT #{node.gate_goal_counter}")
            self.action_goal = node.get_goal(gate_goal[0],gate_goal[1])

        super(GoGate, self).initialise()

    def update(self):
        status = super(GoGate, self).update()

        if status == py_trees.Status.SUCCESS:
            rospy.logwarn("Gate goal reached!")

            if node.gate_goal_counter == 2:
                rospy.logwarn("Gate passed!")    

                node.flag = True
                node.post_counter += 2
                node.spiral_points_counter = 0        
                        
                node.visited_posts.add(node.gate1_id)
                node.visited_posts.add(node.gate2_id)

                try:
                    node.detected_posts.pop(node.gate1_id)
                except:
                    pass

                try:
                    node.detected_posts.pop(node.gate2_id)
                except:
                    pass

                
                py_trees.blackboard.Blackboard().set("remaining_points", len(node.points))
                #py_trees.blackboard.Blackboard().set("remaining_points", 0) # for test
                py_trees.blackboard.Blackboard().set("target_goal", False)

                py_trees.blackboard.Blackboard().set("post_goal", False)

                node.mission_completed()

            else:
                node.gate_goal_counter += 1 
                status = py_trees.Status.SUCCESS

        elif status == py_trees.Status.RUNNING:
            if node.gps_changed():
                return py_trees.Status.FAILURE

        elif status == py_trees.Status.FAILURE:
            rospy.logwarn("Cannot reach gate!")

        return status