#!/home/nvidia/Documents/dora-rover/venv/bin/python3
# license removed for brevity
import rospy
from mavros_msgs.msg import PositionTarget, OverrideRCIn
from geometry_msgs.msg import TwistStamped
from dora import Node
import numpy as np
import time

node = Node()

TARGET_SPEED = 1100


def talker():
    pub = rospy.Publisher("mavros/rc/override", OverrideRCIn, queue_size=1)
    rospy.init_node("talker", anonymous=True)
    for input_id, value, metadata in node:
        [angle] = np.frombuffer(value)
        target = OverrideRCIn()
        if angle < np.pi / 2 and angle > -np.pi / 2:
            target_rotation = int((angle + np.pi / 2) / (np.pi) * 1000) + 1000
            target.channels[0] = target_rotation
            target.channels[2] = TARGET_SPEED
        elif angle < -np.pi / 2:
            target_rotation = 1000
            target.channels[0] = target_rotation
            target.channels[1] = TARGET_SPEED
        else:
            target.channels[0] = 2000
            target.channels[1] = TARGET_SPEED

        # target.channels[2] = 100
        # target = PositionTarget()
        # target.coordinate_frame = 9
        # target.header.stamp = rospy.get_rostime()
        # target.type_mask = int("110111111100",2)
        # target.position.x = 0.9
        # target.position.y = -0.9
        # target.velocity.x = 0.1
        # target.velocity.y = -0.1
        # target.yaw = yaw
        pub.publish(target)
    print("stopping")
    target = OverrideRCIn()
    target.channels[0] = 2000
    target.channels[1] = 120

    pub.publish(target)
    print("stopped")

    # rate.sleep()


if __name__ == "__main__":
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
