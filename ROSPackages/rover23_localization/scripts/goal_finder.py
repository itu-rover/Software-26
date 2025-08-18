#!/usr/bin/env python3
import rospy
import click
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Point, Quaternion, Twist, Vector3, PoseStamped, TransformStamped, Transform
from fiducial_msgs.msg import FiducialTransformArray
import tf 
import tf2_ros
from std_msgs.msg import String, Float32, Int32
from visualization_msgs.msg import MarkerArray, Marker
from math import atan2, cos, sin, pi

class GoalFinder():
    def __init__(self, chosen_waypoints:list):
        rospy.init_node('goal_finder')

        self.rate = rospy.Rate(10)

        rospy.Subscriber('/fiducial_transforms', FiducialTransformArray, self.fiducial_cb)
        rospy.Subscriber('/liorf/mapping/odometry', Odometry, self.odom_cb)
        rospy.Subscriber('/theta', Float32, callback=self.theta_cb)
        rospy.Subscriber('/last_id', Int32, callback=self.id_cb)

        self.marker_pub = rospy.Publisher('/waypoint_markers', MarkerArray, queue_size=100)
        self.pos_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=100)


        self.landmarks = {51:Point(2.009,-5.96,0), 52:Point(7.469,3.632,0), 53:Point(5.984,-2.464,0), 54:Point(12.066,-4.163,0), 55:Point(15.088,1.854,0),     #aruco_id:Point
                          56:Point(22.246,1.73,0), 57:Point(22.268,-3.205,0), 58:Point(26.711,-6.863,0), 59:Point(21.371,-7.905,0), 60:Point(18.039,-3.876,0),
                          61:Point(14.024,-6.598,0), 62:Point(13.19,-11.656,0), 64:Point(6.67,-13.864,0), 65:Point(1.249,-13.804,0)}
        self.waypoints = chosen_waypoints
        self.visited = []
        self.pos = Point(0,0,0)
        self.yaw = 0.0
        self.goalpos = Point(0,0,0)
        self.index = 0
        self.broadcaster = tf2_ros.TransformBroadcaster()
        self.tf_buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tf_buffer) 
        self.aruco_listener = tf.TransformListener()
        self.markers = MarkerArray()
        self.id = 0
        self.theta = 0.0

        self.run()


    def fiducial_cb(self, msg:FiducialTransformArray):
        if len(msg.transforms) != 0:
            trans_1 = TransformStamped()
            trans_1.transform.translation = Vector3(msg.transforms[0].transform.translation.z, -msg.transforms[0].transform.translation.x, msg.transforms[0].transform.translation.y)
            trans_1.transform.rotation = Quaternion(msg.transforms[0].transform.rotation.x, msg.transforms[0].transform.rotation.y, msg.transforms[0].transform.rotation.z, msg.transforms[0].transform.rotation.w)
            trans_1.header.stamp = rospy.Time.now()
            trans_1.child_frame_id = "fiducial_" + str(msg.transforms[0].fiducial_id)
            trans_1.header.frame_id = 'base_link'
            self.broadcaster.sendTransform(trans_1)
        
            if msg.transforms[0].fiducial_id not in self.visited:
                string_id = "fiducial_" + str(msg.transforms[0].fiducial_id)
                if self.tf_buffer.can_transform('odom',string_id,rospy.Time(0)):
                    trans:TransformStamped = self.tf_buffer.lookup_transform('odom',string_id,rospy.Time(0))

                    rot = self.aruco_listener.lookupTransform('/base_link', '/odom', rospy.Time(0))[1]
                    gamma = tf.transformations.euler_from_quaternion((rot[0], rot[1], rot[2], rot[3]))[2]

                    aruco = Point()
                    aruco.x = (msg.transforms[0].transform.translation.z * 0.9 * cos(gamma)) + (-msg.transforms[0].transform.translation.x * 0.9 * sin(gamma))
                    aruco.y = (msg.transforms[0].transform.translation.z * 0.9 * -sin(gamma)) + (-msg.transforms[0].transform.translation.x * 0.9 * cos(gamma))

                    # offset = Point()
                    # landmark_pos = self.landmarks[msg.transforms[0].fiducial_id]

                    # offset.x = ((((self.waypoints[self.index].x * 1.05)- landmark_pos.x) * cos(self.theta)) + (((self.waypoints[self.index].y * 1.05) - landmark_pos.y) * sin(self.theta)))
                    # offset.y = ((((self.waypoints[self.index].x * 1.05) - landmark_pos.x) * -sin(self.theta)) + (((self.waypoints[self.index].y * 1.05) - landmark_pos.y) * cos(self.theta)))

                    self.goalpos.x = self.pos.x + aruco.x #+ offset.x #+ (trans.transform.translation.x * 0.85) 
                    self.goalpos.y = self.pos.y + aruco.y #+ offset.y #+ (trans.transform.translation.y * 0.85) 
                    self.visited.append(msg.transforms[0].fiducial_id)
                    self.publish_markers()
                    self.index += 1

    def odom_cb(self, msg:Odometry):
        self.pos = msg.pose.pose.position
        self.yaw = tf.transformations.euler_from_quaternion((msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w))[2]
        # rospy.loginfo((self.yaw * 180 / 3.1415))


    def theta_cb(self, msg:Float32):
        self.theta = msg.data

    def id_cb(self, msg:Int32):
        self.id = msg.data + 1

    def publish_markers(self):
        self.markers.markers = []
        marker = Marker()
        marker.header.frame_id = 'odom' 
        marker.action = marker.DELETEALL
        self.markers.markers.append(marker)
        self.marker_pub.publish(self.markers)
        rospy.sleep(1.)

        self.markers.markers = []
        marker = Marker()
        marker.header.frame_id = 'odom' 
        marker.action = marker.DELETEALL
        marker.type = marker.CUBE
        marker.action = marker.ADD 
        marker.scale.x = 0.2
        marker.scale.y = 0.2 
        marker.scale.z = 0.2 
        marker.color.a = 1 
        marker.color.r = 0 
        marker.color.g = 1 
        marker.color.b = 0 
        marker.pose.position.x = self.goalpos.x
        marker.pose.position.y = self.goalpos.y
        marker.pose.position.z = 0 
        marker.pose.orientation.x = 0
        marker.pose.orientation.y = 0
        marker.pose.orientation.z = 0
        marker.pose.orientation.w = 1
        marker.id = self.id
        self.id += 1
        
        self.markers.markers.append(marker)
        self.marker_pub.publish(self.markers)
        self.set_goal(self.goalpos.x, self.goalpos.y)

    def set_goal(self, x, y):
        yaw = atan2(y-self.pos.y, x-self.pos.x) 
        goal_quaternion = tf.transformations.quaternion_from_euler(0, 0, yaw)

        goal = PoseStamped()
        goal.header.frame_id = 'odom' 
        goal.pose.position.x = x
        goal.pose.position.y = y
        goal.pose.orientation.x = goal_quaternion[0]
        goal.pose.orientation.y = goal_quaternion[1]
        goal.pose.orientation.z = goal_quaternion[2]
        goal.pose.orientation.w = goal_quaternion[3]

        self.pos_pub.publish(goal)
        rospy.logwarn(f"Going to x = {self.goalpos.x} y = {self.goalpos.y}")

    def run(self):
        while not rospy.is_shutdown():
            # self.publish_markers()

            self.rate.sleep()


@click.command()
@click.option('--wp', prompt='Waypoints', help='Waypoints') #usage -> ... goal_finder.py --wp "1 2 3"
def cli_main(wp:String):
    chosen_wps = []
    wp = wp.split()
    for i in wp:
        chosen_wps.append(waypoint_list[int(i)])
    GoalFinder(chosen_wps)


if __name__ == '__main__':
    waypoint_list = (Point(17.228,-11.549,0), Point(13.485,2.259,0), Point(6.433,-10.337,0), Point(10.874,-7.58,0), Point(19.515,-6.71,0), Point(25.405,-4.851,0), Point(9.454,-0.211,0), Point(19.867,-0.83,0), Point(23.409,-9.082,0), Point(16.031,-3.435,0))
    cli_main()
    
    #rosrun rover23_localization goal_finder.py --wp "0"

