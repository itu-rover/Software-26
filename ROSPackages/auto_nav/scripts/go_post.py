import py_trees_ros
import py_trees
import rospy
import math
from node_config import node

# go to post goal
class GoPost(py_trees_ros.actions.ActionClient):

    def initialise(self):
        """
        Get pos of post, if not visited go to post
        """            
        if node.gps_changed():
            return py_trees.Status.FAILURE

        # currently detected non-visited post
        
        post_goal = py_trees.blackboard.Blackboard().get("post_goal") # get (id,coordinates)

        if post_goal:
            if post_goal[0] in node.detected_posts.keys():
                rospy.logwarn("Going Post %d"%post_goal[0])
                self.action_goal = node.get_goal(post_goal[1][0],post_goal[1][1]) # get (id,coordinates)
                node.navigation_status_pub.publish(f"GOING POST {post_goal[0]}")

        super(GoPost, self).initialise()

    def update(self):
        """
        On success, wait for gps 
        """

        if node.gps_changed():
            return py_trees.Status.FAILURE
            
        status = super(GoPost, self).update()
        
        if status == py_trees.Status.SUCCESS:
            rospy.logwarn("Post reached!")    

            node.flag = True

            py_trees.blackboard.Blackboard().set("target_goal", False)

            node.mission_completed()


        elif status == py_trees.Status.FAILURE:
            rospy.logwarn("Cannot reach post!")

        return status
