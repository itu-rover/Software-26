#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix
# from clap_b7_driver.msg import ClapIns, ClapHeading, ClapGpsPos

rospy.init_node("origin_publisher")


def fix_callback(data:NavSatFix):
    goal = PoseStamped()
    goal.pose.position.x = data.latitude
    goal.pose.position.y = data.longitude
    goal.pose.position.z = 0 
    
    goal.header.seq = 1
    goal.header.stamp = rospy.Time.now()
    goal.header.frame_id = "odom"

    # goal.pose.position.x = data.latitude
    # goal.pose.position.y = data.longitude
    # goal.pose.position.z = 0 #data.pose.pose.position.z

    goal.pose.orientation.x = 0 #data.pose.pose.orientation.x
    goal.pose.orientation.y = 0 #data.pose.pose.orientation.y
    goal.pose.orientation.z = 0 #data.pose.pose.orientation.z
    goal.pose.orientation.w = 1 #data.pose.pose.orientation.w

    rospy.loginfo(f"\n\n Lat-->  {data.latitude}\n Long--> {data.longitude}\n")
    goal_publisher.publish(goal)

goal_publisher = rospy.Publisher("/local_xy_origin", PoseStamped, queue_size=5)
# clap_sub = rospy.Subscriber("clap/clap_ins",ClapIns,callback=clap_callback)
# ublox_sub = rospy.Subscriber("ublox_gps/fix",NavSatFix,callback=fix_callback)
ublox_sub = rospy.Subscriber("gps/fix",NavSatFix,callback=fix_callback)
rospy.spin()



