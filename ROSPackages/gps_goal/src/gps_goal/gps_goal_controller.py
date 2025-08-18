#!/usr/bin/python3
import rospy
import click
import math
import actionlib
import tf
import time
import matplotlib.pyplot as plt

from geographiclib.geodesic import Geodesic
from geometry_msgs.msg import PoseStamped, Twist, Point
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry
from std_msgs.msg import String, Float64MultiArray, Float32
import tf.transformations
from std_msgs.msg import Int32

def DMS_to_decimal_format(lat,long):
  # Check for degrees, minutes, seconds format and convert to decimal
  if ',' in lat:
    degrees, minutes, seconds = lat.split(',')
    degrees, minutes, seconds = float(degrees), float(minutes), float(seconds)
    if lat[0] == '-': # check for negative sign
      minutes = -minutes
      seconds = -seconds
    lat = degrees + minutes/60 + seconds/3600
  if ',' in long:
    degrees, minutes, seconds = long.split(',')
    degrees, minutes, seconds = float(degrees), float(minutes), float(seconds)
    if long[0] == '-': # check for negative sign
      minutes = -minutes
      seconds = -seconds
    long = degrees + minutes/60 + seconds/3600

  lat = float(lat)
  long = float(long)
  rospy.loginfo('Given GPS goal: lat %s, long %s.' % (lat, long))
  return lat, long

def get_origin_lat_long():
  # Get the lat long coordinates of our map frame's origin which must be published on topic /local_xy_origin. We use this to calculate our goal within the map frame.
  # rospy.loginfo("Waiting for a message to initialize the origin GPS location...")
  origin_pose:PoseStamped = rospy.wait_for_message('local_xy_origin', PoseStamped)
  origin_lat = origin_pose.pose.position.x
  origin_long = origin_pose.pose.position.y
  # rospy.loginfo('Received origin: lat %s, long %s.' % (origin_lat, origin_long))
  return origin_lat, origin_long

class GpsGoal():
  def __init__(self, is_aruco):
    rospy.init_node('gps_goal')
    rospy.loginfo("Connecting to move_base...")

    self.move_base = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    self.move_base.wait_for_server()
    self.odom = Odometry()
    self.pos = Point()
    self.rate = rospy.Rate(10)
    self.fix = NavSatFix()
    self.nav_vel = Twist()

    self.yaw = 0
    self.can_go = 1
    self.stop = 0
    self.done = 0
    self.is_oscillating = 0
    self.cog = 0
    self.initial_yaw = 0
    self.is_aligned = 0
    self.is_aruco = is_aruco
    self.detect_signs = 1
    self.error_history = []
    self.oscillation_override = 0 
    print(self.is_aruco)

    self.curent_time = rospy.Time.now()
    self.last_time = rospy.Time.now()

    rospy.loginfo("Connected.")

    rospy.Subscriber('/ublox/odom', Odometry, self.odom_cb)
    rospy.Subscriber("/gps/fix",NavSatFix,callback=self.fix_cb)
    rospy.Subscriber("/nav_vel",Twist,callback=self.nav_vel_cb)
    rospy.Subscriber("/konum/taha", Twist, self.pos_cb)
  

    self.status_pub = rospy.Publisher("/drive_system/status", String, queue_size=10)
    self.spiral_pub = rospy.Publisher("/spiral_start", Float32, queue_size=10)
    self.cmd_pub = rospy.Publisher("/drive_system/twist", Twist, queue_size=10)
    self.pos_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=100)
    self.switch_pub = rospy.Publisher("/switch", Int32, queue_size=10)

    self.can_start = rospy.wait_for_message("/odom_start", Float32)
    

    # Get the lat long coordinates of our map frame's origin which must be publshed on topic /local_xy_origin. We use this to calculate our goal within the map frame.
    self.origin_lat, self.origin_long = get_origin_lat_long()


  def odom_cb(self, msg:Odometry):
    self.odom = msg
    self.pos = msg.pose.pose.position
    self.yaw = tf.transformations.euler_from_quaternion((self.odom.pose.pose.orientation.x, self.odom.pose.pose.orientation.y, self.odom.pose.pose.orientation.z, self.odom.pose.pose.orientation.w))[2]

  def fix_cb(self, msg:NavSatFix):
    self.fix = msg

  def nav_vel_cb(self, msg:Twist):
    self.nav_vel = msg

  def pos_cb(self, msg):
    self.stop = 1
    

  def calc_goal(self, origin_lat, origin_long, goal_lat, goal_long):
    # Calculate distance and azimuth between GPS points
    geod = Geodesic.WGS84  # define the WGS84 ellipsoid
    g = geod.Inverse(origin_lat, origin_long, goal_lat, goal_long) # Compute several geodesic calculations between two GPS points 
    hypotenuse = g['s12']
    azimuth = g['azi1']
    azimuth = math.radians(azimuth)
    x1 = math.cos(azimuth) * hypotenuse
    y1 = -math.sin(azimuth) * hypotenuse

    x2 = x1*math.cos(self.cog) - y1*math.sin(self.cog)
    y2 = x1*math.sin(self.cog) + y1*math.cos(self.cog)

    # x = x2*math.cos(self.initial_yaw) - y2*math.sin(-self.initial_yaw)
    # y = x2*math.sin(-self.initial_yaw) + y2*math.cos(self.initial_yaw)
    
    return x2, y2

  def do_gps_goal(self, goal_lat, goal_long, z=0, yaw=0, roll=0, pitch=0):
    if self.can_start:
      # Initialize plot and plot variables
      t_points = []
      x_o_points = []
      x_pos_points = []
      x_goal_points = []
      x_goal_points_lower = []
      x_goal_points_upper = []

      y_o_points = []
      y_pos_points = []
      y_goal_points = []
      y_goal_points_lower = []
      y_goal_points_upper = []
      
      plt.ion()  # Enable interactive mode
      fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 8))
      line_x_o, = ax1.plot([], [], label="x_o", color="blue")
      line_x_2, = ax1.plot([], [], label="x_2", color="orange")
      line_x_2_upper, = ax1.plot([], [], "--", label="x_2_upper", color="red")
      line_x_2_lower, = ax1.plot([], [], "--", label="x_2_lower", color="red")
      line_self_pos_x, = ax1.plot([], [], label="self.pos.x", color="green")
      line_y_o, = ax2.plot([], [], label="y_o", color="blue")
      line_y_2, = ax2.plot([], [], label="y_2", color="orange")
      line_y_2_upper, = ax2.plot([], [], "--", label="y_2_upper", color="red")
      line_y_2_lower, = ax2.plot([], [], "--", label="y_2_lower", color="red")
      line_self_pos_y, = ax2.plot([], [], label="self.pos.y", color="green")
      
      ax1.set_title("x-axis Data")
      ax2.set_title("y-axis Data")
      ax1.set_xlim(0, 10)  
      ax1.set_ylim(-1, 1)
      ax2.set_xlim(0, 10)  
      ax2.set_ylim(-1, 1)
      ax1.legend()
      ax2.legend()
      ax1.grid()
      ax2.grid()
      velocity = 0.8

      initial_odom:Odometry = rospy.wait_for_message("/ublox/odom", Odometry)
      self.initial_yaw = tf.transformations.euler_from_quaternion([initial_odom.pose.pose.orientation.x,initial_odom.pose.pose.orientation.y,initial_odom.pose.pose.orientation.z,initial_odom.pose.pose.orientation.w])[2]

      initial_cog = rospy.wait_for_message("/initial_cog", Float32)
      self.cog = initial_cog.data*math.pi/180
      

      start_time = time.time()
      x_2, y_2 = self.calc_goal(self.origin_lat, self.origin_long, goal_lat, goal_long)  #x_2 -> x_goal, y_2 -> y_goal

      initial_point = initial_odom.pose.pose.position
      self.pos = initial_odom.pose.pose.position
      x_2 += initial_point.x
      y_2 += initial_point.y

      last_error = [0,0]
      I_x = 0
      I_y = 0
      x_offset = 0 
      y_offset = 0
      x_o = 0
      y_o = 0
      e_x = 0
      e_y = 0

      initial_distance = math.sqrt(((x_2 - self.pos.x)**2)+((y_2 - self.pos.y)**2))
      x_distance = abs(x_2 - self.pos.x)
      y_distance = abs(y_2 - self.pos.y)

      K_i_x = (0.15) * (math.exp(-0.034*10 * x_distance))
      K_i_y = (0.15) * (math.exp(-0.034*10 * y_distance))
      
      self.status_pub.publish('AUTO')
      self.align_to_goal(x_2, y_2)
      
      while (not rospy.is_shutdown()):
          self.curent_time = rospy.Time.now()
          dt = (self.curent_time - self.last_time).to_sec()
          if self.can_go and self.is_aligned:
            self.can_go = 0 if ((abs(self.odom.twist.twist.linear.x) <= 0.01 and abs(self.nav_vel.angular.z) >= 0.7) and ((time.time() - start_time) < 10)) else 1

            self.origin_lat, self.origin_long = get_origin_lat_long()
            x_2, y_2 = self.calc_goal(self.origin_lat, self.origin_long, goal_lat, goal_long)
            x_2 += self.pos.x
            y_2 += self.pos.y
            # rospy.logerr(initial_cog.data)

            e_x = x_2 - self.pos.x

            e_y = y_2 - self.pos.y
          
            I_x += (e_x*(x_distance/initial_distance)) * dt
            # D_x = (e_x - last_error[0]) / dt

            I_y += (e_y*(y_distance/initial_distance)) * dt
            # D_y = (e_y - last_error[1]) / dt

            x_o = (e_x)*((0.1)*(0.8/velocity)) + (K_i_x*I_x) + initial_point.x + x_offset #+ (0.02*D_x)   #controller signal for x-axis
            y_o = (e_y)*((0.1)*(0.8/velocity)) + (K_i_y*I_y) + initial_point.y + y_offset #+ (0.02*D_y)   #controller signal for y-axis

            rospy.loginfo(f"\n\n Goal Lat   --> {goal_lat}\n Current Lat--> {self.origin_lat}\n\n Goal Long   --> {goal_long}\n Current Long--> {self.origin_long}\n\n x_o = {x_o} \n x_2 = {x_2} \n x_1 = {self.pos.x} \n\n y_o = {y_o} \n y_2 = {y_2} \n y_1 = {self.pos.y} \n\n Distance = {math.sqrt((e_x**2)+(e_y**2))} \n (x={round(e_x,4)} y={round(e_y,4)})\n")

            # last_error = [e_x, e_y]

          else:
            self.can_go = 0 if (abs(self.odom.twist.twist.linear.x) <= 0.01 and abs(self.nav_vel.angular.z) >= 0.7) and ((time.time() - start_time) < 10) else 1
            rospy.logwarn("Can't Go")

          # Follow (x_o) and (y_o) with some distance
          if ((not (abs((x_o - self.pos.x) <= 0.88))) or (not (abs(y_o - self.pos.y) <= 0.88))) and (not self.is_oscillating):
            self.publish_goal(x=(x_o), y=(y_o), z=z, yaw=yaw, roll=roll, pitch=pitch)
            
          # Reset I_x and I_y if the error is small and bounce (zıp-zıp)
          if ((abs(x_o - self.pos.x) <= 2)) and (not (abs(e_x) <= 0.88)):
            x_offset += (K_i_x*I_x) + ((e_x)*0.1)
            I_x = 0          
            rospy.logwarn("I_x reset")
          if ((abs(y_o - self.pos.y) <= 2)) and (not (abs(e_y) <= 0.88)):
            y_offset += (K_i_y*I_y) + ((e_y)*0.1)
            I_y = 0
            rospy.logwarn("I_y reset")

          if self.detect_signs:
            self.error_history.append((e_x, e_y))
            if len(self.error_history) > 200:
                self.error_history.pop(0)

          # Detect oscillation
          osc_x = self.detect_sign_oscillation(self.error_history, axis='x')
          osc_y = self.detect_sign_oscillation(self.error_history, axis='y')

          if osc_x and osc_y:
              rospy.logwarn("Oscillation detected in {}!".format('x' if osc_x else 'y'))
              self.oscillation_override = 1
              rospy.logerr("⚠️ Oscillation Override Activated!")

          self.last_time = self.curent_time
          

          # Update Plot
          t_points.append((time.time() - start_time))
          x_o_points.append(x_o)
          x_goal_points.append(x_2)
          x_goal_points_upper.append(x_2+0.88)
          x_goal_points_lower.append(x_2-0.88)
          x_pos_points.append(self.pos.x)

          y_o_points.append(y_o)
          y_goal_points.append(y_2)
          y_goal_points_upper.append(y_2+0.88)
          y_goal_points_lower.append(y_2-0.88)
          y_pos_points.append(self.pos.y)

          line_x_o.set_data(t_points, x_o_points)
          line_x_2.set_data(t_points, x_goal_points)
          line_x_2_upper.set_data(t_points, x_goal_points_upper)
          line_x_2_lower.set_data(t_points, x_goal_points_lower)
          line_self_pos_x.set_data(t_points, x_pos_points)

          line_y_o.set_data(t_points, y_o_points)
          line_y_2.set_data(t_points, y_goal_points)
          line_y_2_upper.set_data(t_points, y_goal_points_upper)
          line_y_2_lower.set_data(t_points, y_goal_points_lower)
          line_self_pos_y.set_data(t_points, y_pos_points)

          ax1.set_xlim(0, max(t_points)+3 if t_points else 10) 
          ax1.set_ylim(min(min(x_o_points)-3, min(x_goal_points_lower)-3, min(x_pos_points)-3),
                      max(max(x_o_points)+3, max(x_goal_points_upper)+3, max(x_pos_points)+3))
          ax2.set_xlim(0, max(t_points)+3 if t_points else 10) 
          ax2.set_ylim(min(min(y_o_points)-3, min(y_goal_points_lower)-3, min(y_pos_points)-3),
                      max(max(y_o_points)+3, max(y_goal_points_upper)+3, max(y_pos_points)+3))

          # fig.canvas.draw_idle()
          # fig.canvas.start_event_loop(0.001)

          # Goal has been reached
          if (((abs(goal_lat-self.origin_lat)<=0.000008) and (abs(goal_long-self.origin_long)<=0.000008)) or (math.sqrt((e_x**2)+(e_y**2)) <= 0.88)) and (self.is_aligned) or self.oscillation_override:
            self.publish_goal(x=self.pos.x, y=self.pos.y, z=z, yaw=yaw, roll=roll, pitch=pitch)
            
            if (not self.stop):
              rospy.logwarn("REACHED GOAL")
              self.can_go = 0
              self.spiral_pub.publish(Float32(1.0))
              self.switch_pub.publish(Int32(0))

            if (not self.is_aruco):
              self.status_pub.publish('SUCCESS')
              self.switch_pub.publish(Int32(0))



            plt.ioff()
            plt.show()
            break
          
          self.rate.sleep()
       

  def publish_goal(self, x, y, z, yaw, roll, pitch):
    quaternion = tf.transformations.quaternion_from_euler(roll, pitch, yaw)

    goal = PoseStamped()
    goal.header.frame_id = 'odom'
    goal.pose.position.x = x
    goal.pose.position.y = y
    goal.pose.orientation.x = quaternion[0]
    goal.pose.orientation.y = quaternion[1]
    goal.pose.orientation.z = quaternion[2]
    goal.pose.orientation.w = quaternion[3]
    if (not self.stop) and self.is_aligned:
      self.pos_pub.publish(goal)


  def align_to_goal(self, x_dist, y_dist):
    angle = math.atan2(y_dist, x_dist)  # Calculate the angle to the goal in radians
    while not self.is_aligned:
      if abs((angle*180/math.pi) - self.yaw*180/math.pi) <= 30:
        self.is_aligned = 1
        rospy.loginfo("Aligned to goal.")
        self.switch_pub.publish(Int32(1))
        vel_command = Twist()
        self.cmd_pub.publish(vel_command)
      elif not self.is_aligned:
        self.is_aligned = 0
        rospy.logwarn("Not aligned to goal, rotating to align.")
        vel_command = Twist()
        vel_command.angular.z = 0.8 if ((angle*180/math.pi) - (self.yaw*180/math.pi)) > 0 else -0.8
        self.cmd_pub.publish(vel_command)
        # time.sleep(2)


  # Detect oscillation using angular.z sign flips.
  def detect_oscillation(self, window_size=20, min_flips=10):
    pass
    # if not hasattr(self, 'angular_history'):
    #   self.angular_history = []

    # self.angular_history.append(self.nav_vel.angular.z)

    # if len(self.angular_history) > window_size:
    #   self.angular_history.pop(0)

    # signs = np.sign(self.angular_history)
    # flips = np.sum(np.diff(signs) != 0)

    # if flips >= min_flips:
    #   rospy.logerr("⚠️ Detected oscillation in angular velocity!")
    #   self.is_oscillating = 1
    #   self.nav_pub.publish(Twist(Vector3(0, 0, 0), Vector3(0, 0, 0.8)))
    # self.is_oscillating = 0 

  def detect_sign_oscillation(self, error_history, axis='x', window_size=200, flip_threshold=3):
    """
    Detects frequent sign changes (oscillations) in e_x or e_y.
    
    Args:
        error_history (list of tuples): List of (e_x, e_y) errors over time.
        axis (str): 'x' or 'y' — which axis to check.
        window_size (int): Number of recent values to check.
        flip_threshold (int): Minimum number of sign changes to consider it oscillation.
        
    Returns:
        bool: True if oscillation is detected.
    """
    if axis not in ('x', 'y'):
        raise ValueError("Axis must be 'x' or 'y'")

    values = [e[0] if axis == 'x' else e[1] for e in error_history[-window_size:]]
    signs = [1 if v > 0 else -1 if v < 0 else 0 for v in values]

    # Count sign changes (ignoring zeros)
    flips = 0
    for i in range(1, len(signs)):
        if signs[i] != 0 and signs[i] != signs[i - 1]:
            flips += 1

    return flips >= flip_threshold


@click.command()
@click.option('--lat', prompt='Latitude', help='Latitude')
@click.option('--long', prompt='Longitude', help='Longitude')
@click.option('--aruco', help='Aruco', default=0.0)
@click.option('--roll', '-r', help='Set target roll for goal', default=0.0)
@click.option('--pitch', '-p', help='Set target pitch for goal', default=0.0)
@click.option('--yaw', '-y', help='Set target yaw for goal', default=0.0)
def cli_main(lat, long, aruco, roll, pitch, yaw):
  """Send goal to move_base given latitude and longitude

  \b
  Two usage formats:
  gps_goal_controller.py --lat 43.658 --long -79.379 # decimal format        yakin        rosrun gps_goal gps_goal_controller.py --lat 39.6628797 --long -110.8568183 
  gps_goal_controller.py --lat 43,39,31 --long -79,22,45 # DMS format        uzak         rosrun gps_goal gps_goal_controller.py --lat 39.6628976 --long -110.857000 
                                                                             aruco        rosrun gps_goal gps_goal_controller.py --lat 39.6629092 --long -110.8568063 --aruco "1"
  """
  gpsGoal = GpsGoal(int(aruco))

  # Check for degrees, minutes, seconds format and convert to decimal
  lat, long = DMS_to_decimal_format(lat, long)
  gpsGoal.do_gps_goal(lat, long, roll=roll, pitch=pitch, yaw=yaw)


if __name__ == '__main__':
  cli_main()



"""
0	1.0	        111 km
1	0.1	        11.1 km
2	0.01	      1.11 km
3	0.001	      111 m
4	0.0001	    11.1 m
5	0.00001	    1.11 m
6	0.000001	  111 mm
7	0.0000001	  11.1 mm
8	0.00000001	1.11 mm
"""
