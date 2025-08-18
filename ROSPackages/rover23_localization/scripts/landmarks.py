#!/usr/bin/env python3
import rospy
import click
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point
from std_msgs.msg import String, Float32, Int32
from visualization_msgs.msg import MarkerArray, Marker
from math import cos, sin, pi

class Landmarks():
    def __init__(self, chosen_waypoints:list):
        rospy.init_node('landmarks')

        self.rate = rospy.Rate(10)

        rospy.Subscriber('/liorf/mapping/odometry', Odometry, self.odom_cb)
        rospy.Subscriber('/theta', Float32, callback=self.theta_cb)

        self.marker_pub = rospy.Publisher('/landmark_markers', MarkerArray, queue_size=100)
        self.id_pub = rospy.Publisher('/last_id', Int32, queue_size=100)


        self.landmarks = {51:Point(2.009,-5.96,0), 52:Point(7.469,3.632,0), 53:Point(5.984,-2.464,0), 54:Point(12.066,-4.163,0), 55:Point(15.088,1.854,0),     #aruco_id:Point
                          56:Point(22.246,1.73,0), 57:Point(22.268,-3.205,0), 58:Point(26.711,-6.863,0), 59:Point(21.371,-7.905,0), 60:Point(18.039,-3.876,0),
                          61:Point(14.024,-6.598,0), 62:Point(13.19,-11.656,0), 64:Point(6.67,-13.864,0), 65:Point(1.249,-13.804,0)}
        
        self.waypoints = chosen_waypoints
        self.positions = []
        self.types = []
        self.markers = MarkerArray()
        self.pos = Point(0,0,0)
        self.start_offset = (Point(17.228,-11.549,0))
        self.theta = 0.0
        self.pub_id = Int32()
        self.done = 0

        self.run()


    def odom_cb(self, msg:Odometry):
        self.pos = msg.pose.pose.position


    def theta_cb(self, msg:Float32):
        self.theta = msg.data

    def input(self):
        print("Awaiting input:")
        input1 = int(input())
        if input1 == 1: self.done = 1
        print("Publishing Landmarks")



    def calculate_positions(self):
        rospy.logwarn(f"THETA === {self.theta}")
        for i in self.landmarks:
            current_offset = Point(0,0,0)
            current_offset.x = ((self.landmarks[i].x - self.start_offset.x) * cos(self.theta)) + ((self.landmarks[i].y - self.start_offset.y) * sin(self.theta)) #+ self.pos.x
            current_offset.y = ((self.landmarks[i].x - self.start_offset.x) * -sin(self.theta)) + ((self.landmarks[i].y - self.start_offset.y) * cos(self.theta)) #+ self.pos.y
            self.positions.append(current_offset)
            self.types.append(1)

        for i in self.waypoints:
            current_offset = Point(0,0,0)
            current_offset.x = ((i.x - self.start_offset.x) * cos(self.theta)) + ((i.y - self.start_offset.y) * sin(self.theta))
            current_offset.y = ((i.x - self.start_offset.x) * -sin(self.theta)) + ((i.y - self.start_offset.y) * cos(self.theta))
            self.positions.append(current_offset)
            self.types.append(0)

        rospy.logwarn(self.positions)
        self.publish_markers()
            

    def publish_markers(self):
        self.markers.markers = []
        marker = Marker()
        marker.header.frame_id = 'odom' 
        marker.action = marker.DELETEALL

        self.markers.markers.append(marker)
        self.marker_pub.publish(self.markers)
        rospy.sleep(1.)


        self.markers.markers = []
        for i in range(len(self.positions)):
            marker = Marker()
            marker.header.frame_id = 'odom' 
            # marker.action = marker.DELETEALL
            marker.action = marker.ADD 
            marker.scale.x = 0.5
            marker.scale.y = 0.5 
            marker.scale.z = 0.5 
            if self.types[i] == 1: #aruco
                marker.type = marker.CYLINDER
                marker.color.a = 1 
                marker.color.r = 0.89
                marker.color.g = 0.24
                marker.color.b = 0.58
            else:
                marker.type = marker.SPHERE
                marker.color.a = 1 
                marker.color.r = 1
                marker.color.g = 0.65 
                marker.color.b = 0
            # print(self.positions[i].x)
            marker.pose.position.x = self.positions[i].x
            marker.pose.position.y = self.positions[i].y
            marker.pose.position.z = 0 
            marker.pose.orientation.x = 0
            marker.pose.orientation.y = 0
            marker.pose.orientation.z = 0
            marker.pose.orientation.w = 1
            marker.id = i
            
            self.markers.markers.append(marker)

            self.pub_id.data = i
            

        # self.marker_pub.publish(self.markers)
    def run(self):
        while not rospy.is_shutdown():
            if not self.done:
                self.input()
                self.calculate_positions()
                
            self.marker_pub.publish(self.markers)
            self.id_pub.publish(self.pub_id)
            self.rate.sleep()


@click.command()
@click.option('--wp', prompt='Waypoints', help='Waypoints') #usage -> ... landmarks.py --wp "1 2 3"
def cli_main(wp:String):
    chosen_wps = []
    wp = wp.split()
    for i in wp:
        chosen_wps.append(waypoint_list[int(i)])
    Landmarks(chosen_wps)

if __name__ == '__main__':
    waypoint_list = (Point(17.228,-11.549,0), Point(13.485,2.259,0), Point(6.433,-10.337,0), Point(10.874,-7.58,0), Point(19.515,-6.71,0), Point(25.405,-4.851,0), Point(9.454,-0.211,0), Point(19.867,-0.83,0), Point(23.409,-9.082,0), Point(16.031,-3.435,0))
    cli_main()
    
