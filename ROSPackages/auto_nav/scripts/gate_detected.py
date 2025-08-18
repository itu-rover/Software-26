import py_trees
import tf
import math
import rospy
from node_config import node

# check if a gate is detected
class GateDetected(py_trees.behaviour.Behaviour):

    def update(self):
        
        if node.gps_changed():
            return py_trees.Status.FAILURE

        if len(node.gate_coordinates) == 2: # Calculate middle point back, mid and front

            node.stop(1.0)
            
            first_pole = list(node.gate_coordinates.items())[0]
            second_pole = list(node.gate_coordinates.items())[1]

            mid_point_x = (first_pole[1][0] + second_pole[1][0])/2
            mid_point_y = (first_pole[1][1] + second_pole[1][1])/2
            mid_point_z = (first_pole[1][2] + second_pole[1][2])/2

            #converting quaternion to rpy
            orientation_list_1 = [first_pole[1][3], first_pole[1][4], first_pole[1][5], first_pole[1][6]]
            (roll_1, pitch_1, yaw_1) = tf.transformations.euler_from_quaternion(orientation_list_1)
            orientation_list_2 = [second_pole[1][3] , second_pole[1][4],second_pole[1][5],second_pole[1][6]]
            (roll_2, pitch_2, yaw_2) = tf.transformations.euler_from_quaternion(orientation_list_2)

            if (roll_1*roll_2<0):
                roll_1=abs(roll_1)
                roll_2=abs(roll_2)

            #calculating average of rpy values
            roll=(roll_1+roll_2)/2.0
            pitch=(pitch_1+pitch_2)/2.0
            yaw=(yaw_1+yaw_2)/2.0
            node.broadcaster.sendTransform((mid_point_x, mid_point_y, mid_point_z),
            tf.transformations.quaternion_from_euler(roll+math.pi, pitch, yaw),
            rospy.Time.now(),
            "middle_point",
            "odom")
            goal_mid = (mid_point_x, mid_point_y)

            node.broadcaster.sendTransform((mid_point_x + 4*math.cos(yaw_1), mid_point_y + 4*math.sin(yaw_1), mid_point_z),
            tf.transformations.quaternion_from_euler(roll+math.pi, pitch, yaw),
            rospy.Time.now(),
            "middle_point_back",
            "odom")
            goal_back = (mid_point_x + 4*math.cos(yaw_1),mid_point_y + 4*math.sin(yaw_1))

            node.broadcaster.sendTransform((mid_point_x - 4*math.cos(yaw_1), mid_point_y - 4*math.sin(yaw_1), mid_point_z),
            tf.transformations.quaternion_from_euler(roll+math.pi, pitch, yaw),
            rospy.Time.now(),
            "middle_point_front",
            "odom")
            goal_front = (mid_point_x - 4*math.cos(yaw_1), mid_point_y - 4*math.sin(yaw_1))

            py_trees.blackboard.Blackboard().set("gate_goals",[goal_back,goal_mid,goal_front]) # (back,mid,front)

            return py_trees.common.Status.SUCCESS

        return py_trees.common.Status.FAILURE
