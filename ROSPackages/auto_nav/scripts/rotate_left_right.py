import py_trees
import rospy
from node_config import node, node_config

class RotateLeft(py_trees.behaviour.Behaviour):

    def update(self):
        node.rotate(node_config.ROTATE_SPEED)
        node.update_gate()
        #rospy.loginfo("Rotating left")
        node.navigation_status_pub.publish("ROTATING LEFT")

        return py_trees.common.Status.SUCCESS  

class RotateRight(py_trees.behaviour.Behaviour):
    
    def update(self):
        node.rotate(-node_config.ROTATE_SPEED)
        node.update_gate()
        #rospy.loginfo("Rotating right")
        node.navigation_status_pub.publish("ROTATING RIGHT")

        return py_trees.common.Status.SUCCESS 