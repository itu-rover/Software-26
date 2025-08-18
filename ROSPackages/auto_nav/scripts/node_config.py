import rospy
from move_base_spiral import MoveBaseSpiral

class NodeConfig:
    def __init__(self, TICK_TOCK, POST_NUMBER, CIRCLE_RADIUS, ROTATE_SPEED):
        self.TICK_TOCK = TICK_TOCK
        self.POST_NUMBER = POST_NUMBER
        self.CIRCLE_RADIUS = CIRCLE_RADIUS
        self.ROTATE_SPEED = ROTATE_SPEED

    def print(self, attrs):
        print("NODE CONFIG PARAMETERS:")
        print("-----------------------")
        print('\n'.join("%s: %s" % item for item in attrs.items()))

TICK_TOCK = 100

POST_NUMBER = 5

CIRCLE_RADIUS = 7.5

ROTATE_SPEED = 0.3

node_config = NodeConfig(TICK_TOCK, POST_NUMBER, CIRCLE_RADIUS, ROTATE_SPEED)
attrs = vars(node_config)
node_config.print(attrs)

rospy.init_node("auto_nav_node")

node = MoveBaseSpiral()

#while(node.gps_flag): pass # wait for start points