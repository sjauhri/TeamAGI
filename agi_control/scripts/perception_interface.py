# Import modules
import rospy

from agi_control.msg import StackedCubesMSG

class StackedCubesPub():
    def __init__(self):
        self.pub = rospy.Publisher('StackedCubesInfo',
                                    StackedCubesMSG,
                                    queue_size=10)

    def __call__(self, location, n_stacked):
        # generate msg
        msg = StackedCubesMSG()
        msg.location = location
        msg.n_stacked = n_stacked
        # publish
        self.pub.publish(msg)
        