#!/usr/bin/env python2

# library -> system-specific parameters and functions
import sys

# library -> pure python client library for ROS
import rospy

# library -> Moveit python functions for ROS
import moveit_commander

# library -> message types representing primitive data types
# It must be Float32 by Number -> js
from std_msgs.msg import Float32

# library -> services such as BatteryState, CameraInfo, and etc.
from sensor_msgs.msg import JointState


class Robot_Control(object):
    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node("TESTOS2", anonymous=True)
        self.move_group = moveit_commander.MoveGroupCommander("manipulator")

    def go_to_joint(self, data):
        joint_goal = self.move_group.get_current_joint_values()

        if "#base" in data.name:
            joint_goal[0] = data.position[0]

        elif "#shoulder" in data.name:
            joint_goal[1] = data.position[0]

        elif "#elbow" in data.name:
            joint_goal[2] = data.position[0]

        elif "#wrist_1" in data.name:
            joint_goal[3] = data.position[0]

        elif "#wrist_2" in data.name:
            joint_goal[4] = data.position[0]

        elif "#wrist_3" in data.name:
            joint_goal[5] = data.position[0]
        else:
            pass

        if "no_wait" in data.name:
            self.move_group.go(joint_goal, wait=False)
            self.move_group.stop()

        else:
            self.move_group.go(joint_goal, wait=True)
            self.move_group.stop()

    def set_accvel(self, data):
        self.move_group.set_max_velocity_scaling_factor(data.data / 100)
        self.move_group.set_max_acceleration_scaling_factor(data.data / 100)

    def listener(self):
        rospy.Subscriber("action_joint_data", JointState, self.go_to_joint)
        rospy.Subscriber("set_vel_acc", Float32, self.set_accvel)
        rospy.spin()


def control_robot():
    robot = Robot_Control()

    while not rospy.is_shutdown():
        robot.listener()


if __name__ == "__main__":
    control_robot()
