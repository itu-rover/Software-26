#!/usr/bin/python3
import rospy
import math
import actionlib
import tf

from geographiclib.geodesic import Geodesic
from actionlib_msgs.msg import GoalStatus
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_srvs.srv import Trigger, TriggerResponse
from sensor_msgs.msg import NavSatFix, Imu
from nav_msgs.msg import Odometry
from gps_goal.srv import *

from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3



class Odom():
  def __init__(self):
    rospy.init_node('odom_publisher')

    rospy.Service("/start_pose", Trigger, self.start)
    rospy.Service("/end_pose", Trigger, self.end)
    rospy.Subscriber("ublox_gps/fix", NavSatFix, self.gps_curr_fix_callback)
    
    self.odom_pub = rospy.Publisher("odom/burak", Odometry, queue_size=50)
    self.odom_broadcaster = tf.TransformBroadcaster()

    self.start_lat = 0
    self.start_lng = 0

    self.end_lat = 0
    self.end_lng = 0

    self.curr_lat = 0
    self.curr_lng = 0

    self.initialized = False
    self.initial_azi = 0
    self.x = 0
    self.y = 0
    self.yaw = 0
    self.curr_imu_yaw = 0
    self.accumulator = 0

    self.rate = rospy.Rate(10)

    self.start_time = rospy.Time.now().to_sec()

    rospy.Subscriber("clap/ros/imu", Imu, self.imu_callback)

    while not rospy.is_shutdown():
      if not self.initialized:
        continue
      
      self.loop()
      delta_time = (rospy.Time.now().to_sec()-self.start_time)
      rospy.loginfo(delta_time)
      self.accumulator = (delta_time) * (0.00038241591853072 - 0.0004393796155850733 + 0.00008980784743518225)
      self.rate.sleep()


  def calc_goal(self, curr_lat, curr_long, goal_lat, goal_long, verbose=False):
    # Calculate distance and azimuth between GPS points
    geod = Geodesic.WGS84  # define the WGS84 ellipsoid
    g = geod.Inverse(curr_lat, curr_long, goal_lat, goal_long) # Compute several geodesic calculations between two GPS points 
    hypotenuse = distance = g['s12'] # access distance
    azimuth = g['azi2']

    if verbose:
      rospy.loginfo("The distance from the curr to the goal is {:.3f} m.".format(distance))
      rospy.loginfo("The azimuth from the curr to the goal is {:.3f} degrees.".format(azimuth))
      rospy.loginfo("The imu yaw from the curr to the goal is {:.3f} degrees.".format(self.curr_imu_yaw*180./3.14))

    return distance, g['azi1'], g['azi2']
    
  def gps_curr_fix_callback(self, data):
    self.curr_lat = data.latitude
    self.curr_lng = data.longitude

  def imu_callback(self, data):
    orientation = data.orientation
    orientation_list = [orientation.x, orientation.y, orientation.z, orientation.w]
    (roll, pitch, self.curr_imu_yaw) = tf.transformations.euler_from_quaternion(orientation_list)
    # rospy.loginfo(self.curr_imu_yaw)

  def start(self, data):
    rospy.loginfo("start")
    self.start_lat = self.curr_lat
    self.start_lng = self.curr_lng

  def end(self, data):
    rospy.loginfo("end")
    self.end_lat = self.curr_lat
    self.end_lng = self.curr_lng
    distance, azi1, self.initial_azi = self.calc_goal(self.start_lat, self.start_lng, self.end_lat, self.end_lng, True)
    self.initial_imu_yaw = self.curr_imu_yaw
    self.initialized = True

    self.start_time = rospy.Time.now().to_sec()
  
  def loop(self):
    distance, azi1, azi2 = self.calc_goal(self.end_lat, self.end_lng, self.curr_lat, self.curr_lng)

    azi1 = math.radians(azi1-self.initial_azi)
    self.x = math.cos(azi1) * distance
    self.y =  -math.sin(azi1) * distance
    # self.yaw = self.initial_azi - (self.initial_imu_yaw-self.curr_imu_yaw)*180/3.14
    self.yaw = self.curr_imu_yaw - self.initial_imu_yaw + self.accumulator
    rospy.loginfo(f"{self.x} {self.y} {self.yaw}")

    current_time = rospy.Time.now()
    # since all odometry is 6DOF we'll need a quaternion created from yaw
    odom_quat = tf.transformations.quaternion_from_euler(0, 0, self.yaw)

    # first, we'll publish the transform over tf
    self.odom_broadcaster.sendTransform(
        (self.x, self.y, 0.),
        odom_quat,
        current_time,
        "base_link",
        "odom"
    )

    # next, we'll publish the odometry message over ROS
    odom = Odometry()
    odom.header.stamp = current_time
    odom.header.frame_id = "odom"

    # set the position
    odom.pose.pose = Pose(Point(self.x, self.y, 0.), Quaternion(*odom_quat))

    # set the velocity
    odom.child_frame_id = "base_link"

    # publish the message
    self.odom_pub.publish(odom)

def ros_main():
  gpsGoal = Odom()
  

if __name__ == '__main__':
  ros_main()
