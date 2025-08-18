#!/usr/bin/env python
# -- coding: utf-8 --

#written by ahmetuyuklu
#edited by payzun
#edited by berat

import rospy
import py_trees
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
import math
import tf
from nav_msgs.msg import Odometry
from actionlib_msgs.msg import GoalID 
from geometry_msgs.msg import Twist, TransformStamped 
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Bool, String
from visualization_msgs.msg import Marker,MarkerArray
import tf2_ros
from fiducial_msgs.msg import FiducialTransformArray
import traceback
from geographiclib.geodesic import Geodesic
from auto_nav.srv import * 
from std_srvs.srv import SetBool, SetBoolResponse
from actionlib_msgs.msg import GoalStatus
from std_msgs.msg import Float32, Int32

# this class is used global variable named "node" representing the navigated rover.

class MoveBaseSpiral:
    def __init__(self):
    
        self.flag = True
        self.gps_flag = True # update when post and pole reached
        self.stop_flag = False

        self.poles = []
        self.gate1_id = -1
        self.gate2_id = -1
        self.gate_goal_counter = 0
        self.points = [] # spiral points
        self.circle_points = [] # circle points for searching gate
        self.markers = MarkerArray()
        self.fiducial_seconds = 0  # to avoid redundant timestamp warning for saved fiducials
        self.move = 1

        rospy.Subscriber('/ublox/odom', Odometry,self.datapos)  # CHANGE ODOMETRY TOPIC
        rospy.Subscriber('/fiducial_transforms', FiducialTransformArray,self.fiducial_cb)
        rospy.Subscriber('/ublox/fix', NavSatFix,self.gps_cb)  # change this to ublox/fix when testing
        rospy.Subscriber("/konum/taha", Twist, self.pos_cb)

        self.spiral_points_counter = 0 # counter for visited points
        self.post_counter = 0 # counter for visited posts
        self.circle_point_counter = 0 # counter for circle points

        self.visited_posts = set() 
        self.detected_posts = dict() # objects stored as key = fiducial_id, value = (x,y) -> (key,value)

        self.gate_coordinates = dict()  

        self.tf_buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tf_buffer) 
        self.broadcaster = tf.TransformBroadcaster()

        self.mission_service = rospy.Service('mission', Mission, self.handle_mission_service) # added for x, y goals
        self.gps_goal_service = rospy.Service('gps_goal', LatLon, self.handle_gps_goal_service)
        self.gps_goal_service = rospy.Service('gps_goal_fix', LatLon, self.handle_gps_goal_fix_callback) # new gps goal added by emre
        self.stop_service = rospy.Service('stop_mission', SetBool, self.handle_stop_service)

        self.marker_pub = rospy.Publisher("/spiral_markers", MarkerArray, queue_size = 10) # create markers
        self.stop_pub = rospy.Publisher("/e_stop", Bool, queue_size = 100) # velocity publisher
        self.goal_abort_pub = rospy.Publisher("/move_base/cancel", GoalID, queue_size=1) # abort current goal
        self.rotate_pub = rospy.Publisher("/rover_velocity_controller/cmd_vel", Twist, queue_size = 100) # velocity publisher
        
        # for navigation status in interface
        self.navigation_status_pub = rospy.Publisher("/navigation_status", String, queue_size=1) 
        

        rospy.loginfo("Connecting to move_base...")
        self.move_base = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.move_base.wait_for_server()
        rospy.loginfo("Connected.")
        self.can_go = rospy.wait_for_message("/spiral_start", Float32)
        self.switch_pub = rospy.Publisher("/switch", Int32, queue_size=10)

        rospy.loginfo("Can go")
        rospy.sleep(1.)

    def gps_changed(self):
        gps_changed = py_trees.blackboard.Blackboard().get("gps_changed")
        if gps_changed:
            return True
        return False
    
    def pos_cb(self, msg):
        self.move = 0
        self.abort_goal()

    def mission_completed(self):
        py_trees.blackboard.Blackboard().set("gps_xy", False)
        self.delete_all_markers()
        rospy.logwarn("Mission completed!")
        self.navigation_status_pub.publish("MISSION COMPLETED")

    def reset(self, stop=False):
        self.detected_posts = dict()
        self.visited_posts = set()
        self.gate_coordinates = dict()
        self.poles = list()
        self.gate1_id = -1
        self.gate2_id = -1
        self.gate_goal_counter = 0
        self.circle_points = []
        self.circle_point_counter = 0
        self.points = [] # spiral points
        self.spiral_points_counter = 0
        self.create_points(1,25,math.pi/4, True)

        if stop:
            self.stop_flag = True

        py_trees.blackboard.Blackboard().set("gps_reached", False)
        py_trees.blackboard.Blackboard().set("gps_xy", False)
        py_trees.blackboard.Blackboard().set("gps_changed", True)
        py_trees.blackboard.Blackboard().set("remaining_points", len(self.points))
        py_trees.blackboard.Blackboard().set("target_goal", False)
        py_trees.blackboard.Blackboard().set("preempted", False)

    def begin_mission(self):
        self.reset()
        rospy.logwarn("New mission started!")

    def handle_stop_service(self, data):
        if data.data:
            self.reset(stop=True)
            # self.abort_goal()  # doesnt work because of locomove
            self.stop(1.0)  # stop for 1 sec
            rospy.logwarn("Aborted mission!")
        
        return SetBoolResponse(True, "Aborted mission!")

    def handle_mission_service(self, req):  # for x, y goals
        
        self.begin_mission()

        print("Given parameters: ", (req.x, req.y, req.poles))
        
        py_trees.blackboard.Blackboard().set("gps_xy", (req.x, req.y))
        # py_trees.blackboard.Blackboard().set("gps_changed", True)
        
        self.poles = list(req.poles)
                
        if len(self.poles) == 2: # determine the id of the gates
            self.gate1_id = self.poles[0]
            self.gate2_id = self.poles[1]

        return MissionResponse("gps goal is given with x and y coordinates")

    def handle_gps_goal_service(self, req):
        
        self.begin_mission()

        print("Given parameters: ", (req.latitude, req.longitude, req.poles))
        
        # for gps goal
        x, y = self.calc_gps_goal(req.latitude, req.longitude)
        py_trees.blackboard.Blackboard().set("gps_xy", (x, y))
        # py_trees.blackboard.Blackboard().set("gps_changed", True)

        self.poles = list(req.poles)
                
        if len(self.poles) == 2: # determine the id of the gates
            self.gate1_id = self.poles[0]
            self.gate2_id = self.poles[1]

        return LatLonResponse("gps goal is given")

    def calc_goal(self, curr_lat, curr_long, goal_lat, goal_long):
        # Calculate distance and azimuth between GPS points
        geod = Geodesic.WGS84  # define the WGS84 ellipsoid
        g = geod.Inverse(curr_lat, curr_long, goal_lat, goal_long) # Compute several geodesic calculations between two GPS points 
        hypotenuse = distance = g['s12'] # access distance
        rospy.loginfo("The distance from the curr to the goal is {:.3f} m.".format(distance))
        azimuth = g['azi1']
        rospy.loginfo("The azimuth from the curr to the goal is {:.3f} degrees.".format(azimuth))

        # Convert polar (distance and azimuth) to x,y translation in meters (needed for ROS) by finding side lenghs of a right-angle triangle
        # Convert azimuth to radians
        azimuth = math.radians(azimuth)
        delta_x = math.cos(azimuth) * hypotenuse
        delta_y =  math.sin(azimuth) * hypotenuse
        rospy.loginfo("The translation from the origin to the goal is (x,y) {:.3f}, {:.3f} m.".format(delta_x, delta_y))

        return delta_x, -delta_y

    def do_gps_goal(self, goal_lat, goal_long, z=0, yaw=0, roll=0, pitch=0):
        self.delta_x, self.delta_y = self.calc_goal(self.curr_lat, self.curr_long, goal_lat, goal_long)
        goal_x = self.curr_x + self.delta_x
        goal_y = self.curr_y + self.delta_y 
        self.publish_goal(x=goal_x, y=goal_y, z=z, yaw=yaw, roll=roll, pitch=pitch)

    def handle_gps_goal_fix_callback(self, data):
        rospy.loginfo("Yeni goal geldi")
        self.goal_lat = data.latitude
        self.goal_long = data.longitude
        self.is_new_goal = False
        rospy.sleep(3)
        self.is_new_goal = True
        self.do_gps_goal(self.goal_lat, self.goal_long)
        while self.is_new_goal:
            if abs(self.delta_x) < 10 and abs(self.delta_y) < 10:
                rospy.loginfo("bitti")
                return LatLonResponse("gittik goale bitti is")
            
            # fark azalınca hızlı hızlı goal verirsek recovery behavior'a giriyor
            elif abs(self.delta_x) < 20 or abs(self.delta_y) < 20:
                self.do_gps_goal(self.goal_lat, self.goal_long)
                rospy.sleep(3)
            else:
                self.do_gps_goal(data.latitude, data.longitude)
                rospy.sleep(0.35)

    def publish_goal(self, x=0, y=0, z=0, yaw=0, roll=0, pitch=0):
        # Create move_base goal
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "odom"
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        goal.target_pose.pose.position.z =  z
        quaternion = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
        goal.target_pose.pose.orientation.x = quaternion[0]
        goal.target_pose.pose.orientation.y = quaternion[1]
        goal.target_pose.pose.orientation.z = quaternion[2]
        goal.target_pose.pose.orientation.w = quaternion[3]
        rospy.loginfo('Executing move_base goal to position (x,y) %s, %s, with %s degrees yaw.' %
                (x, y, yaw))
        rospy.loginfo("To cancel the goal: 'rostopic pub -1 /move_base/cancel actionlib_msgs/GoalID -- {}'")

        # Send goal
        if self.move:
            self.move_base.send_goal(goal)
            rospy.loginfo('Inital goal status: %s' % GoalStatus.to_string(self.move_base.get_state()))
            status = self.move_base.get_goal_status_text()
        if status:
            rospy.loginfo(status)

        # Wait for goal result
        self.move_base.wait_for_result()
        rospy.loginfo('Final goal status: %s' % GoalStatus.to_string(self.move_base.get_state()))
        status = self.move_base.get_goal_status_text()
        if status:
            rospy.loginfo(status)


    def calc_gps_goal(self, goal_lat, goal_long):
        
        origin_lat = self.start_point_lat
        origin_long = self.start_point_lon

        #origin_lat = 41.1052672
        #origin_long = 29.0233784

        # Calculate distance and azimuth between GPS points
        geod = Geodesic.WGS84  # define the WGS84 ellipsoid
        g = geod.Inverse(origin_lat, origin_long, goal_lat, goal_long) # Compute several geodesic calculations between two GPS points 
        hypotenuse = distance = g['s12'] # access distance
        rospy.loginfo("The distance from the origin to the goal is {:.3f} m.".format(distance))
        azimuth = g['azi1']
        rospy.loginfo("The azimuth from the origin to the goal is {:.3f} degrees.".format(azimuth))

        # Convert polar (distance and azimuth) to x,y translation in meters (needed for ROS) by finding side lenghs of a right-angle triangle
        # Convert azimuth to radians
        azimuth = math.radians(azimuth)
        x = adjacent = math.cos(azimuth) * hypotenuse
        y = opposite = math.sin(azimuth) * hypotenuse
        rospy.loginfo("The translation from the origin to the goal is (x,y) {:.3f}, {:.3f} m.".format(x, y))

        return -y, x

    def abort_goal(self):
        cancel_msg = GoalID()
        self.goal_abort_pub.publish(cancel_msg)

    def create_points(self, start, end, angle_step,init = False): # create spiral points.
        self.switch_pub.publish(Int32(1))
        try:
            if init:
                for r in range(start, end+1):
                    theta = angle_step * r
                    x = (r+1) * math.cos(theta)/2
                    y = (r+1) * math.sin(theta)/2

                    self.points.append((x+self.curr_x, y+self.curr_y))

            self.create_markers(False)
            rospy.loginfo("Spiral points has been created")

        except Exception:
            rospy.logwarn('Spiral points cannot created.')
            traceback.print_exc()

    def delete_all_markers(self):

        self.markers.markers = []
            
        marker = Marker()
        marker.header.frame_id = 'odom'
        marker.action = marker.DELETEALL
        
        self.markers.markers.append(marker)
        self.marker_pub.publish(self.markers)
        rospy.sleep(1.)


    def create_markers(self, arr_circle):

        self.delete_all_markers()

        self.markers.markers = []
        arr = []
        circle_x = 0.0
        circle_y = 0.0

        if arr_circle:
            arr = self.circle_points
        else:
            arr = self.points

        for i in arr:
            marker = Marker()
            marker.header.frame_id = 'odom' #以哪一个TF坐标为原点
            marker.type = marker.CUBE #一直面向屏幕的字符格式
            marker.action = marker.ADD #添加marker
            marker.scale.x = 0.25 #marker大小
            marker.scale.y = 0.25 #marker大小
            marker.scale.z = 0.25 #marker大小，对于字符只有z起作用
            marker.color.a = 1 #字符透明度
            marker.color.r = 1 #字符颜色R(红色)通道
            marker.color.g = 1 #字符颜色G(绿色)通道
            marker.color.b = 0 #字符颜色B(蓝色)通道
            if arr_circle:
                marker.pose.position.x = i[0] #字符位置
                marker.pose.position.y = i[1] #字符位置
            else:
                marker.pose.position.x = self.start_point_x + i[0] #字符位置
                marker.pose.position.y = self.start_point_y + i[1] #字符位置

            marker.pose.position.z = 0 #msg.position.z #字符位置
            marker.pose.orientation.x = 0
            marker.pose.orientation.y = 0
            marker.pose.orientation.z = 0#字符位置
            marker.pose.orientation.w = 0 #字符位置
            self.markers.markers.append(marker)
        id = 0
        for m in self.markers.markers:
            m.id = id
            id += 1

    def find_shortest_index(self):
        
        first_point = self.circle_points[0]
        shortest_dist = (self.x_cur - first_point[0]) ** 2 + (self.y_cur - first_point[1]) ** 2 
        shortest_index = 0
        
        for i in range(1,len(self.circle_points)):
            point = self.circle_points[i]
            d = (self.x_cur - point[0]) ** 2 + (self.y_cur - point[1]) ** 2 
            if d < shortest_dist:
                shortest_dist = d
                shortest_index = i
        
        print("shortest index: ", shortest_index)
        return shortest_index

    def calculate_circle_points(self, r, center_x, center_y):

        self.circle_points = []
        self.circle_points.append((center_x+r,center_y))
        self.circle_points.append((center_x,center_y+r))
        self.circle_points.append((center_x-r,center_y))
        self.circle_points.append((center_x,center_y-r))

        self.shortest_index = self.find_shortest_index()

        self.create_markers(True)
        rospy.loginfo("Circle points has been created")
        print(self.circle_points)

    def datapos(self,data):
        self.curr_x = data.pose.pose.position.x
        self.curr_y = data.pose.pose.position.y
        if self.flag:
            self.start_point_x = data.pose.pose.position.x
            self.start_point_y = data.pose.pose.position.y
            print("Start point is updated as: " + str(self.start_point_x) + ", " + str(self.start_point_y))
            self.flag = False
        
        # Save the current x,y position
        self.x_cur = data.pose.pose.position.x
        self.y_cur = data.pose.pose.position.y
        euler = tf.transformations.euler_from_quaternion([data.pose.pose.orientation.x,data.pose.pose.orientation.y,data.pose.pose.orientation.z,data.pose.pose.orientation.w])
        self.yaw = euler[2]

    def gps_cb(self,data):
    
        if self.gps_flag:
            self.start_point_lat = data.latitude
            self.start_point_lon = data.longitude
            print("Start point for GPS is updated as: " + str(self.start_point_lat) + ", " + str(self.start_point_lon))
            self.gps_flag = False
        
        self.curr_lat = data.latitude
        self.curr_long = data.longitude
        
    def rotate(self, angle):
        rotate_twist = Twist()
        rotate_twist.linear.x = 0
        rotate_twist.linear.y = 0
        rotate_twist.linear.z = 0

        rotate_twist.angular.x = 0
        rotate_twist.angular.y = 0
        rotate_twist.angular.z = angle

        self.rotate_pub.publish(rotate_twist)

    def publish_markers(self):

        #while self.flag:
            #pass

        self.marker_pub.publish(self.markers)

    def fiducial_cb(self,fiducials):

        #self.detected_posts = dict()

        if self.fiducial_seconds == 0:
            self.fiducial_seconds = rospy.get_time()
        
        now = rospy.get_time()
        if now - self.fiducial_seconds > 1.5: # every 1.5 sec refresh tf
            self.fiducial_seconds = now

            for k,v in self.gate_coordinates.items():
                fid_id = "fid_" + str(k) + "_saved" 

                self.broadcaster.sendTransform((v[0],v[1],v[2]),
                        (v[3],v[4],v[5],v[6]),
                        rospy.Time.now(), 
                        fid_id,
                        "odom")

        for fiducial in fiducials.transforms:
            if fiducial.fiducial_id not in self.visited_posts:
                string_id = "fiducial_" + str(fiducial.fiducial_id)
                if self.tf_buffer.can_transform('odom',string_id,rospy.Time(0)): # transform fiducial coordinates to /odom frame
                    trans = self.tf_buffer.lookup_transform('odom',string_id,rospy.Time(0))
                    self.detected_posts[fiducial.fiducial_id] = [trans.transform.translation.x,trans.transform.translation.y,trans.transform.translation.z,trans.transform.rotation.x,trans.transform.rotation.y,trans.transform.rotation.z,trans.transform.rotation.w]
                else:
                    #self.detected_posts.pop(fiducial.fiducial_id)
                    rospy.logwarn("Cannot transform %s"%string_id)

    def get_goal(self, x, y, rotate_to_center = False):
        self.switch_pub.publish(Int32(1))
        #Calculate the yaw angle
        yaw = math.atan2(y-self.y_cur, x-self.x_cur) 
        goal_quaternion = None

        if rotate_to_center:
            goal_quaternion = tf.transformations.quaternion_from_euler(0, 0, yaw + 90)
        else:
            goal_quaternion = tf.transformations.quaternion_from_euler(0, 0, yaw)

        # Create a move_base goal to /odom topic
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'odom'
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        goal.target_pose.pose.orientation.x = goal_quaternion[0]
        goal.target_pose.pose.orientation.y = goal_quaternion[1]
        goal.target_pose.pose.orientation.z = goal_quaternion[2]
        goal.target_pose.pose.orientation.w = goal_quaternion[3]
        
        print("Goal is x: " + str(x)+" y: "+str(y)+ "\nCurrent position is x: " +str(self.x_cur)+ " y: "+str(self.y_cur)+"\nyaw:\t"+str(yaw*180/math.pi))

        return goal

    def stop(self, time):

        v = True
        self.stop_pub.publish(v)
        rospy.logwarn("Stopped!")
        rospy.sleep(time)
        v = False
        self.stop_pub.publish(v)

    def update_gate(self):
        for k in self.detected_posts.keys():
            if k == self.gate1_id or k == self.gate2_id:
                self.gate_coordinates[k] = self.detected_posts[k]
                if k not in self.gate_coordinates.keys():
                    #self.stop()
                    self.gate_coordinates[k] = self.detected_posts[k]
