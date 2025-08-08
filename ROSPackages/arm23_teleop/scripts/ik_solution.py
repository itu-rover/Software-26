#!/usr/bin/env python3

import rospy
from moveit_commander import RobotCommander
from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest
from geometry_msgs.msg import PoseStamped, Pose
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from std_msgs.msg import Header
from sensor_msgs.msg import JointState

# ROS'u başlat
rospy.init_node('ik_test_node')

# IK servisini başlat
rospy.wait_for_service('/compute_ik')
compute_ik = rospy.ServiceProxy('/compute_ik', GetPositionIK)

# IK isteği oluştur
ik_request = GetPositionIKRequest()
ik_request.ik_request.group_name = "manipulator"

# Hedef pozisyon ve rotasyon
pose_goal = Pose()
pose_goal.position.x = 0 # +x sağ
pose_goal.position.y = 0.3 # +y ileri
pose_goal.position.z = 0.35 # +z yukarı
pose_goal.orientation.w = 1.0
pose_goal.orientation.x = 0.0
pose_goal.orientation.y = 0.0
pose_goal.orientation.z = 0.0

# PoseStamped mesajını oluştur ve pose_goal'u kullan
pose_stamped = PoseStamped()
pose_stamped.header.frame_id = "end_effector"  # Baz çerçevenizi buraya koymalısınız (arm_base, end_effector, ...)
pose_stamped.pose = pose_goal
ik_request.ik_request.pose_stamped = pose_stamped

# IK çözümlerini al
try:
    response = compute_ik(ik_request)
    if response.error_code.val == response.error_code.SUCCESS:
        joint_angles = response.solution.joint_state.position
        print("Joint Angles:", joint_angles)


        positions_publisher = rospy.Publisher('/manipulator_controller/command', JointTrajectory, queue_size=10) # /manipulator_controller/command
        rate = rospy.Rate(10)
        trajectory_msg = JointTrajectory()
        trajectory_msg.header = Header()
        trajectory_msg.header.frame_id = 'arm_base' 
        trajectory_msg.joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
        point = JointTrajectoryPoint()

        point.positions = [0, 0, 0, 0, 0, 0]

        for i in range (6):
            point.positions[i] = joint_angles[i]
        
        point.time_from_start = rospy.Duration(2)
        trajectory_msg.points.append(point)

        while not rospy.is_shutdown():

            trajectory_msg.header.stamp = rospy.Time.now()
            positions_publisher.publish(trajectory_msg)
            #rospy.loginfo("position updated")
            rate.sleep()

    else:
        print("IK çözümü basarisiz.")
except rospy.ServiceException as e:
    print("Hizmet çagrisi basarisiz: %s" % e)
