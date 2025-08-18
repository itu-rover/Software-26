#!/usr/bin/python3

import rospy
import tf
from robot_localization.srv import FromLL
from geographic_msgs.msg import GeoPoint
from geometry_msgs.msg import Point
class Gps_goal():
    def __init__(self):
        rospy.init_node("Goal_node")
        rospy.wait_for_service("fromLL")
        self.llpoint = GeoPoint()
        self.rate = rospy.Rate(50)
        self.goal_broadcaster = tf.TransformBroadcaster()
        self.fromll = rospy.ServiceProxy("/fromLL", FromLL)

        self.map_points = Point()
        self.qua = [0,0,0,1]

    def llservice(self):
        self.llpoint.latitude = -111.0698690
        self.llpoint.longitude = 36.2405189
        self.llpoint.altitude = 0
        try:
            self.map_points = self.fromll(self.llpoint)
            rospy.loginfo(self.map_points)
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s" %e)

    def main(self):
        while not rospy.is_shutdown():
            self.llservice()
            self.goal_broadcaster.sendTransform((self.map_points.x, self.map_points.y, 0),
                                                self.qua,
                                                rospy.Time.now(),
                                                "Target_goal",
                                                "base_link")
            self.rate.sleep()

if __name__ == '__main__':
    try:
        Gps_goal().main()
    except rospy.ROSInterruptException:
        pass
