#!/usr/bin/env python3

import rospy
from std_srvs.srv import Empty

def clear_costmaps():
    rospy.wait_for_service('/move_base/clear_costmaps')
    try:
        clear_costmaps_proxy = rospy.ServiceProxy('/move_base/clear_costmaps', Empty)
        clear_costmaps_proxy()
        rospy.loginfo("Costmaps cleared successfully.")
    except rospy.ServiceException as e:
        rospy.logerr("Failed to clear costmaps: %s" % e)

if __name__ == '__main__':
    rospy.init_node('clear_costmaps_node')
    clear_costmaps()
