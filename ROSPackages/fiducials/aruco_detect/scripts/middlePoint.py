#!/usr/bin/env python3
#written by yerenkl & ahmetuyuklu
import rospy
import tf
from geometry_msgs.msg import Transform, Quaternion
import geometry_msgs.msg
from fiducial_msgs.msg import FiducialTransformArray, FiducialTransform
from math import acos, sqrt, pi

def callback(msg):
    try:
        for n in range(0,len(msg.transforms)):
            if(msg.transforms[n].fiducial_id==2): #arucotag_1 id
                n_0=n
            elif(msg.transforms[n].fiducial_id==3):#arucotag_2 id
                n_1=n
        ax=msg.transforms[n_0].transform.translation.x
        ay=msg.transforms[n_0].transform.translation.y
        az=msg.transforms[n_0].transform.translation.z
        bx=msg.transforms[n_1].transform.translation.x
        by=msg.transforms[n_1].transform.translation.y
        bz=msg.transforms[n_1].transform.translation.z

        x=(ax+bx)/2
        y=(ay+by)/2
        z=(az+bz)/2

        #converting quaternion to rpy
        orientation_q_1 = msg.transforms[n_0].transform.rotation
        orientation_list_1 = [orientation_q_1.x, orientation_q_1.y, orientation_q_1.z, orientation_q_1.w]
        (roll_1, pitch_1, yaw_1) = tf.transformations.euler_from_quaternion(orientation_list_1)
        orientation_q_2 = msg.transforms[n_1].transform.rotation
        orientation_list_2 = [orientation_q_2.x, orientation_q_2.y, orientation_q_2.z, orientation_q_2.w]
        (roll_2, pitch_2, yaw_2) = tf.transformations.euler_from_quaternion(orientation_list_2)


        if (roll_1*roll_2<0):
            roll_1=abs(roll_1)
            roll_2=abs(roll_2)

        #calculating average of rpy values
        roll=(roll_1+roll_2)/2.0
        pitch=(pitch_1+pitch_2)/2.0
        yaw=(yaw_1+yaw_2)/2.0

        #publishing tf
        broadcaster.sendTransform((x, y, z),
                    tf.transformations.quaternion_from_euler(roll+pi, pitch, yaw),
                    rospy.Time.now(),
                    "middle_point",
                    "head_camera")
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