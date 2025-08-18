#!/usr/bin/env python3
#written by yerenkl & ahmetuyuklu & yemreakar
import rospy
import tf
from geometry_msgs.msg import Transform, Quaternion
import geometry_msgs.msg
from fiducial_msgs.msg import FiducialTransformArray, FiducialTransform
from math import acos, sqrt, pi
from std_msgs.msg import Float32MultiArray

def callback(msg):
    try:
        n_0 = msg.transforms[0].fiducial_id
        rospy.loginfo(n_0)
        ax=msg.transforms[n_0].transform.translation.x
        ay=msg.transforms[n_0].transform.translation.y
        az=msg.transforms[n_0].transform.translation.z\

        x= ax
        y= ay
        z= az

        #converting quaternion to rpy
        orientation_q_1 = msg.transforms[n_0].transform.rotation
        orientation_list_1 = [orientation_q_1.x, orientation_q_1.y, orientation_q_1.z, orientation_q_1.w]
        (roll_1, pitch_1, yaw_1) = tf.transformations.euler_from_quaternion(orientation_list_1)


        if (roll_1*roll_2<0):
            roll_1=abs(roll_1)
            roll_2=abs(roll_2)

        #calculating average of rpy values
        roll= roll_1
        pitch= pitch_1
        yaw= yaw_1

        #publishing tf
        broadcaster.sendTransform((z, -x, y),
                    tf.transformations.quaternion_from_euler(roll+pi, pitch, yaw),
                    rospy.Time.now(),
                    ("my_fiducial_" + str(n_0)),
                    "base_link")
        
        """
        print("roll_1: "+str(roll_1)+" pitch_1: "+ str(pitch_1)+" yaw_1: "+str(yaw_1))
        print("roll_2: "+str(roll_2)+" pitch_2: "+ str(pitch_2)+" yaw_2: "+str(yaw_2))
        print("roll: "+str(roll)+" pitch: "+ str(pitch)+" yaw: "+str(yaw))
        """
    except:
        pass

if __name__ == '__main__':
    rospy.init_node('aruco_listener')
    rospy.Subscriber("/fiducial_transforms", FiducialTransformArray, callback=callback)

    broadcaster = tf.TransformBroadcaster()
    while not rospy.is_shutdown():
        continue
