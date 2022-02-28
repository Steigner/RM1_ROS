#!/usr/bin/env python2

# library -> system-specific parameters and functions
import sys

# library -> pure python client library for ROS
import rospy

# library -> Moveit python functions for ROS
import moveit_commander

# library -> geometry_msgs provides messages for common geometric primitives types
from geometry_msgs.msg import PoseStamped


def pub_curr_position(m_group):
    publisher = rospy.Publisher("current_position", PoseStamped, queue_size=10)
    rospy.init_node("POS", anonymous=True)

    while not rospy.is_shutdown():
        current_position = m_group.get_current_pose()
        publisher.publish(current_position)


if __name__ == "__main__":
    moveit_commander.roscpp_initialize(sys.argv)
    m_group = moveit_commander.MoveGroupCommander("manipulator")

    try:
        pub_curr_position(m_group)
    except rospy.ROSInterruptException:
        pass
