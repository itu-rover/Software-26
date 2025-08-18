#!/usr/bin/env python

import rospy
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32
from geographiclib.geodesic import Geodesic
from clap_b7_driver.msg import ClapHeading
import math

class GPSOdometry:
    def __init__(self):
        rospy.init_node('gps_odometry_node', anonymous=True)
        
        # GPS fix subscriber
        self.gps_sub = rospy.Subscriber('/ublox_gps/fix', NavSatFix, self.gps_callback)
        
        # Heading subscriber (assuming heading is published in degrees)
        self.heading_sub = rospy.Subscriber('/clap/heading', ClapHeading, self.heading_callback)
        
        # Odometry publisher
        self.odom_pub = rospy.Publisher('/gps/odom', Odometry, queue_size=10)
        
        self.geod = Geodesic.WGS84
        self.prev_lat = None
        self.prev_lon = None
        self.heading = None
        
        self.odom = Odometry()
        self.odom.header.frame_id = 'odom'
        self.odom.child_frame_id = 'base_link'
        
    def gps_callback(self, msg):
        if self.prev_lat is None or self.prev_lon is None:
            self.prev_lat = msg.latitude
            self.prev_lon = msg.longitude
            return
        
        # Calculate the distance and bearing between two GPS points
        g = self.geod.Inverse(self.prev_lat, self.prev_lon, msg.latitude, msg.longitude)
        distance = g['s12']
        bearing = g['azi1']
        
        # Update position using distance and heading
        if self.heading is not None:
            theta = math.radians(self.heading)
            dx = distance * math.cos(theta)
            dy = distance * math.sin(theta)
            
            self.odom.pose.pose.position.x += dx
            self.odom.pose.pose.position.y += dy
            
            # Set orientation
            q = self.heading_to_quaternion(self.heading)
            self.odom.pose.pose.orientation.x = q[0]
            self.odom.pose.pose.orientation.y = q[1]
            self.odom.pose.pose.orientation.z = q[2]
            self.odom.pose.pose.orientation.w = q[3]
        
        self.odom.header.stamp = rospy.Time.now()
        self.odom_pub.publish(self.odom)
        
        self.prev_lat = msg.latitude
        self.prev_lon = msg.longitude
    
    def heading_callback(self, msg):
        self.heading = msg.heading
    
    def heading_to_quaternion(self, heading):
        theta = math.radians(heading)
        q = [0, 0, math.sin(theta / 2), math.cos(theta / 2)]
        return q

if __name__ == '__main__':
    try:
        GPSOdometry()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

