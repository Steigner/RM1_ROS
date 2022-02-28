#!/usr/bin/env python

""" UNUSED SCRIPT """
""" TEST PURPOSE """

import rospy
import moveit_commander
from math import pi


class ResetRobot(object):
    def __init__(self):
        rospy.init_node("reset_robot")
        group = moveit_commander.MoveGroupCommander("manipulator")

        for i in range(3):
            self.reset(group)
            rospy.sleep(1)

    def reset(self, group):
        joint_target = {
            "shoulder_pan_joint": 0,
            "shoulder_lift_joint": -pi / 2,
            "elbow_joint": 0,
            "wrist_1_joint": 0,
            "wrist_2_joint": 0,
            "wrist_3_joint": 0,
        }

        group.set_joint_value_target(joint_target)
        group.go(wait=True)


if __name__ == "__main__":
    reset = ResetRobot()
