#!/usr/bin/env python

""" UNUSED SCRIPT """
""" TEST PURPOSE """

import rospy
from geometry_msgs.msg import WrenchStamped


def talker():
    pub = rospy.Publisher("HEXSIM", WrenchStamped, queue_size=10)

    rospy.init_node("talker", anonymous=True)

    rate = rospy.Rate(10)  # 10hz

    while not rospy.is_shutdown():
        wrench = WrenchStamped()
        wrench.header.frame_id = "Head"
        wrench.wrench.force.x = 40

        rospy.loginfo(wrench)

        pub.publish(wrench)
        rate.sleep()


if __name__ == "__main__":
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
