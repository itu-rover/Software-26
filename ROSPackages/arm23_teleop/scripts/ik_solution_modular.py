#!/usr/bin/env python3
# 23 mart 2025: çalıştı konuma gitti ve döngüden çıktı

import rospy
import numpy as np
from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest
from geometry_msgs.msg import PoseStamped, Pose, Twist
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import Header
from std_msgs.msg import Bool
from std_msgs.msg import String
from sensor_msgs.msg import JointState
import json
import time

class RobotIKController:
    def __init__(self):
        rospy.init_node("ik_robot_controller")
        self.joint_pub = rospy.Publisher('/manipulator_controller/command', JointTrajectory, queue_size=10)
        self.press_button = rospy.Publisher('/press_button', Bool, queue_size=10)
        self.current_state = np.array([0,0,0,0,0,0], dtype=np.longdouble)

        self.target_word = rospy.get_param("~target_word", "purna")
        self.target_word = self.target_word.upper()
        self.word_length = len(self.target_word)
        self.letter_indices = {}
        for idx, letter in enumerate(self.target_word):
            if letter not in self.letter_indices:
                self.letter_indices[letter] = []
            self.letter_indices[letter].append(idx)
        self.target_position = np.zeros((self.word_length, 3), dtype=np.longdouble)
        self.goal_check = np.zeros((self.word_length), dtype=np.longdouble)
        self.clear_previous_bool = 1

        rospy.Subscriber('/joint_states', JointState, self.joint_state_callback)
        rospy.Subscriber('/dictionary_topic', String, self.dictionary_callback)
        
        rospy.wait_for_service('/compute_ik')
        self.compute_ik = rospy.ServiceProxy('/compute_ik', GetPositionIK)

    def joint_state_callback(self, data):
        self.current_state[0] = data.position[0]
        self.current_state[1] = data.position[1]
        self.current_state[2] = data.position[2]
        self.current_state[3] = data.position[3]
        self.current_state[4] = data.position[4]
        self.current_state[5] = data.position[5]
    
    def dictionary_callback(self, msg):
        try:
            data = json.loads(msg.data)

            for letter, values in data.items():
                if letter in self.letter_indices:
                    x, y, z, ascii_val = values
                    
                    for i in self.letter_indices[letter]:
                        self.target_position[i][0] = x + 0.25 # offsetler ekle
                        self.target_position[i][1] = -y - 0.225
                        self.target_position[i][2] = z + 0.15
                        #rospy.loginfo(f"{letter} harfi {i}. satira yerleştirildi: X={x:.2f}, Y={y:.2f}, Z={z:.2f}")

        except json.JSONDecodeError as e:
            rospy.logerr(f"JSON çözümleme hatasi: {e}")


    def compute_inverse_kinematics(self, x, y, z):
        ik_request = GetPositionIKRequest()
        ik_request.ik_request.group_name = "manipulator"

        pose_goal = Pose()
        pose_goal.position.x = x 
        pose_goal.position.y = y 
        pose_goal.position.z = z 
        pose_goal.orientation.w = 0.707
        pose_goal.orientation.x = 0.0
        pose_goal.orientation.y = -0.707
        pose_goal.orientation.z = 0.0

        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = "arm_base"
        pose_stamped.pose = pose_goal
        ik_request.ik_request.pose_stamped = pose_stamped

        try:
            response = self.compute_ik(ik_request)
            if response.error_code.val == response.error_code.SUCCESS:
                return response.solution.joint_state.position
            else:
                rospy.logwarn("IK çözümü basarisiz.")
                return None
        except rospy.ServiceException as e:
            rospy.logerr(f"IK hizmet çağrisi başarisiz: {e}")
            return None
    
    def send_joint_trajectory(self, joint_angles):
        trajectory_msg = JointTrajectory()
        trajectory_msg.header = Header()
        trajectory_msg.header.frame_id = 'arm_base'
        trajectory_msg.joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
        
        point = JointTrajectoryPoint()
        point.positions = joint_angles
        point.time_from_start = rospy.Duration(2)
        trajectory_msg.points.append(point)

        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            trajectory_msg.header.stamp = rospy.Time.now()
            self.joint_pub.publish(trajectory_msg)
            if np.all(np.allclose(self.current_state, joint_angles, atol=0.15)):
                rospy.loginfo("Hedef pozisyona ulaşildi.")
                break
            rate.sleep()
    
    def solve_and_go(self, x, y, z):
        rospy.loginfo(f"Hedefe gidiliyor: ({x}, {y}, {z})")
        joint_angles = self.compute_inverse_kinematics(x, y, z)
        if joint_angles:
            self.send_joint_trajectory(joint_angles)
            rospy.loginfo("Hedefe ulaşildi!\n")
            self.press_button.publish(True)
            return 1
        else:
            rospy.logwarn("IK çözülemedi, hareket iptal edildi.")
            return 0
    
    def main(self):
        while not rospy.is_shutdown():
            for i in range(self.word_length):
                if i == 0:
                    self.clear_previous_bool = 1
                if self.goal_check[i] == 0 and not np.array_equal(self.target_position[i], [0, 0, 0]):
                    print(f"{i} icin hesaplama yapilacak")
                    for j in range(i):
                        if i-j>=1:
                            if self.goal_check[j] == 1:
                                self.clear_previous_bool = 1
                                continue
                            else:
                                self.clear_previous_bool = 0
                                break
                    if self.clear_previous_bool == 1:
                        print("going to!")     
                        going = self.solve_and_go(self.target_position[i][0], self.target_position[i][1], self.target_position[i][2])
                    if going == 1:
                        self.goal_check[i] = 1
                else:
                    pass

            if np.all(self.goal_check == 1.0):
                print("Kelime yazildi!")
                break   

            time.sleep(0.1)

if __name__ == "__main__":
    try:
        controller = RobotIKController()
        controller.main()
    except rospy.ROSInterruptException:
        pass
