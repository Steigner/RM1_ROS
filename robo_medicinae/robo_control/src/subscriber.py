#!/usr/bin/env python

# library -> pure python client library for ROS
import rospy

# library -> Moveit python functions for ROS
import moveit_commander

# library -> geometry_msgs provides messages for common geometric primitives types
from geometry_msgs.msg import WrenchStamped


class save_prev(object):
    prev = None


class Stop(object):
    def __init__(self):
        self.move_group = moveit_commander.MoveGroupCommander("manipulator")
        rospy.init_node("HEX", anonymous=True)
        rospy.Subscriber("HEXSIM", WrenchStamped, self.callback)
        rospy.spin()

    def callback(self, data):
        wrench = WrenchStamped()
        wrench.header.frame_id = data.header.frame_id
        wrench.wrench.force.x = data.wrench.force.x

        if save_prev.prev == None:
            save_prev.prev = WrenchStamped()
            save_prev.prev.header.frame_id = data.header.frame_id
            save_prev.prev.wrench.force.x = data.wrench.force.x
            save_prev.prev.wrench.force.y = data.wrench.force.y
            save_prev.prev.wrench.force.z = data.wrench.force.z
            print(save_prev.prev)

        else:
            if (
                data.wrench.force.x > save_prev.prev.wrench.force.x + 0.1
                or data.wrench.force.x < save_prev.prev.wrench.force.x - 0.1
            ):
                print(data.wrench.force.x)
                for i in range(10):
                    self.move_group.stop()

            if (
                data.wrench.force.y > save_prev.prev.wrench.force.y + 0.05
                or data.wrench.force.y < save_prev.prev.wrench.force.y - 0.05
            ):
                print(data.wrench.force.y)
                for i in range(10):
                    self.move_group.stop()

            if data.wrench.force.z > save_prev.prev.wrench.force.z + 0.08:
                print(data.wrench.force.z)
                for i in range(10):
                    self.move_group.stop()
