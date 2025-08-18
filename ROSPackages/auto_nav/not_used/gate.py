#!/usr/bin/env python3

import rospy
import tf2_ros
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
import math
import tf
from nav_msgs.msg import Odometry


class tflistener(object):

    def __init__(self):
        self.flag = False
        self.fid14Flag = False
        self.fid18Flag = False

        self.tf_buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tf_buffer)
        rospy.Subscriber('/odometry/wheel', Odometry,self.datapos)

        # Initialize the action client
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.client.wait_for_server()
        rospy.loginfo("Connected to move_base server")
        self.rate = rospy.Rate(1)
        self.broadcaster = tf.TransformBroadcaster()


    def datapos(self,data):
        # Save the current x,y position
        self.x_cur = data.pose.pose.position.x
        self.y_cur = data.pose.pose.position.y
        self.pos_x_cur = data.pose.pose.orientation.x
        self.pos_y_cur = data.pose.pose.orientation.y
        self.pos_z_cur = data.pose.pose.orientation.z
        self.pos_w_cur = data.pose.pose.orientation.w
        self.tf_transform()
    
    def tf_transform(self):
        if self.tf_buffer.can_transform('odom','fiducial_14',rospy.Time(0)):
            trans = self.tf_buffer.lookup_transform('odom','fiducial_14',rospy.Time(0))
            self.tx_fid14 = trans.transform.translation.x
            self.ty_fid14 = trans.transform.translation.y
            self.tz_fid14 = trans.transform.translation.z
            self.rx_fid14 = trans.transform.rotation.x
            self.ry_fid14 = trans.transform.rotation.y
            self.rz_fid14 = trans.transform.rotation.z
            self.rw_fid14 = trans.transform.rotation.w
            self.fid14Flag = True

        else:
            rospy.logdebug("Can't transform fiducial_14.")
        if self.tf_buffer.can_transform('odom','fiducial_18',rospy.Time(0)):
            trans = self.tf_buffer.lookup_transform('odom','fiducial_18',rospy.Time(0))
            self.tx_fid18 = trans.transform.translation.x
            self.ty_fid18 = trans.transform.translation.y
            self.tz_fid18 = trans.transform.translation.z
            self.rx_fid18 = trans.transform.rotation.x
            self.ry_fid18 = trans.transform.rotation.y
            self.rz_fid18 = trans.transform.rotation.z
            self.rw_fid18 = trans.transform.rotation.w
            self.fid18Flag = True

        else:
            rospy.logdebug("Can't transform fiducial_18.")
        if self.fid14Flag and self.fid18Flag:
            self.mid_point_x = (self.tx_fid18 + self.tx_fid14)/2
            self.mid_point_y = (self.ty_fid18 + self.ty_fid14)/2
            self.mid_point_z = (self.tz_fid18 + self.tz_fid14)/2


            #converting quaternion to rpy
            orientation_list_1 = [self.rx_fid18, self.ry_fid18, self.rz_fid18, self.rw_fid18]
            (roll_1, pitch_1, yaw_1) = tf.transformations.euler_from_quaternion(orientation_list_1)
            orientation_list_2 = [self.rx_fid14, self.ry_fid14, self.rz_fid14, self.rw_fid14]
            (roll_2, pitch_2, yaw_2) = tf.transformations.euler_from_quaternion(orientation_list_2)

            if (roll_1*roll_2<0):
                roll_1=abs(roll_1)
                roll_2=abs(roll_2)

            #calculating average of rpy values
            self.roll=(roll_1+roll_2)/2.0
            self.pitch=(pitch_1+pitch_2)/2.0
            self.yaw=(yaw_1+yaw_2)/2.0
            #print(self.mid_point_x, self.mid_point_y,self.mid_point_z)
            #print(roll_1)

            #rospy.logdebug("tx_mid: %s, ty_mid: %s, tz_mid: %s", tx_mid_point, ty_mid_point, tz_mid_point)
            #rospy.logdebug("tx_back: %s, ty_back: %s, tz_back: %s", tx_mid_point_back, ty_mid_point_back, tz_mid_point_back)
            #rospy.logdebug("tx_front: %s, ty_front: %s, tz_front: %s", tx_mid_point_front, ty_mid_point_front, tz_mid_point_front)


    def send_goal(self, x, y):
        #Calculate the yaw angle
        yaw = math.atan2(y-self.y_cur, x-self.x_cur) 
        goal_quaternion = tf.transformations.quaternion_from_euler(0, 0, yaw)

        # Create a move_base goal
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'odom'
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        goal.target_pose.pose.orientation.x = goal_quaternion[0]
        goal.target_pose.pose.orientation.y = goal_quaternion[1]
        goal.target_pose.pose.orientation.z = goal_quaternion[2]
        goal.target_pose.pose.orientation.w = goal_quaternion[3]
        
        print("Goal is x: " + str(x)+" y: "+str(y)+ "\nCurrent position is x: " +str(self.x_cur)+ " y: "+str(self.y_cur)+"\nyaw:\t"+str(yaw*180/math.pi))
        # Send the goal to the action server
        self.client.send_goal(goal)

        # Wait for the result
        self.client.wait_for_result()

        # Print the result
        if self.client.get_state() == actionlib.GoalStatus.SUCCEEDED:
            rospy.loginfo("Goal reached!")
        else:
            rospy.loginfo("Goal failed with error code: " + str(self.client.get_state()))
if __name__ == '__main__':
    rospy.init_node('tf_listener', anonymous=True)#, log_level=rospy.DEBUG)
    node = tflistener()
    while not rospy.is_shutdown():
        node.rate.sleep()
        try:
            if node.tx_mid_point_back !=0:
                print(node.tx_mid_point_back,node.ty_mid_point_back)
                print(node.tx_mid_point,node.ty_mid_point)
                print(node.tx_mid_point_front,node.ty_mid_point_front)
                #node.send_goal(node.tx_mid_point_back,node.ty_mid_point_back)
                #node.send_goal(node.tx_mid_point,node.ty_mid_point)
                #node.send_goal(node.tx_mid_point_front,node.ty_mid_point_front)
                node.rate.sleep()

        except:
            pass
        try:
            node.broadcaster.sendTransform((node.tx_fid18,node.ty_fid18,node.tz_fid18),
                (node.rx_fid18,node.ry_fid18,node.rz_fid18,node.rw_fid18),
                rospy.Time.now(), 
                "fid_18",
                "odom")
        except:
            print("18 olmadi")
            pass
        try:
            node.broadcaster.sendTransform((node.tx_fid14,node.ty_fid14,node.tz_fid14),
                (node.rx_fid14,node.ry_fid14,node.rz_fid14,node.rw_fid14),
                rospy.Time.now(), 
                "fid_14",
                "odom")
        except:
            pass
        try:
            node.broadcaster.sendTransform((node.mid_point_x, node.mid_point_y, node.mid_point_z),
            tf.transformations.quaternion_from_euler(node.roll+math.pi, node.pitch, node.yaw),
            rospy.Time.now(),
            "middle_point",
            "odom")
        except:
            print("mid point olmadi")
            pass
        try:
            node.broadcaster.sendTransform((node.mid_point_x + 2*math.cos(node.yaw), node.mid_point_y + 2*math.sin(node.yaw), node.mid_point_z),
            tf.transformations.quaternion_from_euler(node.roll+math.pi, node.pitch, node.yaw),
            rospy.Time.now(),
            "middle_point_back",
            "odom")
        except:
            print("mid back olmadi")
            pass
        try:
            node.broadcaster.sendTransform((node.mid_point_x - 4*math.cos(node.yaw), node.mid_point_y - 4*math.sin(node.yaw), node.mid_point_z),
            tf.transformations.quaternion_from_euler(node.roll+math.pi, node.pitch, node.yaw),
            rospy.Time.now(),
            "middle_point_front",
            "odom")
        except:
            pass
        #node.send_goal(node.tx_mid_point_back,node.ty_mid_point_back)