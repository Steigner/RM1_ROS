#!/usr/bin/env python2

# library -> subprocess management
import subprocess

# library -> set handlers for asynchronous events
import signal

# library -> Moveit python functions for ROS
import moveit_commander

# library -> pure python client library for ROS
import rospy

# library -> message types representing primitive data types
from std_msgs.msg import String

# script  -> open, close RG2 gripper
from rg2 import open_rg2, close_rg2, IP


class Switch(object):
    # init killer var, killer quit subprocess
    def __init__(self):
        self.killer = None

    # init moveit move group
    def init(self):
        self.move_group = moveit_commander.MoveGroupCommander("manipulator")

    # run python motion_robot init_motion()
    def init_mot(self):
        self.start_R2 = subprocess.Popen(
            ["rosrun", "robo_control", "motion_robot.py", "1"]
        )
        self.killer = "init"

    # quit python motion_robot init_motion()
    def init_mot_kill(self):
        self.start_R2.send_signal(signal.SIGINT)

    # run python motion_robot rotate_motion()
    def rotate_mot(self):
        self.start_R2 = subprocess.Popen(
            ["rosrun", "robo_control", "motion_robot.py", "2"]
        )
        self.killer = "rotate"

    # quit python motion_robot rotate_motion()
    def rotate_mot_kill(self):
        self.start_R2.send_signal(signal.SIGINT)

    # run python motion_robot nostril_init() + HEX subscriber
    def init_nostr(self):
        self.start_R3 = subprocess.Popen(
            ["rosrun", "robo_control", "motion_robot.py", "3"]
        )
        self.start_R3_1 = subprocess.Popen(["rosrun", "robo_control", "subscriber.py"])
        self.killer = "nostr"

    # quit python motion_robot nostril_init() + HEX subscriber
    def init_nostr_kill(self):
        self.start_R3.send_signal(signal.SIGINT)
        self.start_R3_1.send_signal(signal.SIGINT)

    # run python control_robot + position publisher
    def con_rob(self):
        self.start_R4 = subprocess.Popen(["rosrun", "robo_control", "control_robot.py"])
        self.start_R5 = subprocess.Popen(["rosrun", "robo_control", "pose_robot.py"])
        self.killer = "control"

    # quit python control_robot + position publisher
    def con_rob_kill(self):
        self.start_R4.send_signal(signal.SIGINT)
        self.start_R5.send_signal(signal.SIGINT)

    # run ur driver or gazebo simulation + moveit
    def connect(self, ip):
        IP.ip = ip

        if ip == "127.0.0.1":
            self.start_R_1 = subprocess.Popen(
                ["roslaunch", "robo_platform", "ur3_bringup.launch"]
            )
            rospy.sleep(2)
            self.start_R_2 = subprocess.Popen(
                [
                    "roslaunch",
                    "robo_moveit",
                    "ur3_moveit_planning_execution.launch",
                    "sim:=true",
                ]
            )
            rospy.sleep(10)
            self.init()

        else:
            self.start_R_1 = subprocess.Popen(
                [
                    "roslaunch",
                    "robo_platform",
                    "ur3_driver_bringup.launch",
                    "robot_ip:=" + ip,
                ]
            )
            rospy.sleep(2)
            self.start_R_2 = subprocess.Popen(
                ["roslaunch", "robo_moveit", "ur3_moveit_planning_execution.launch"]
            )
            rospy.sleep(10)
            self.init()

    # quit ur driver or gazebo simulation + moveit
    def disconnect(self):
        self.start_R_1.send_signal(signal.SIGINT)
        rospy.sleep(2)
        self.start_R_2.send_signal(signal.SIGINT)

    # emergency stop function
    def emergency_stop(self):
        move_group = self.move_group
        for i in range(10):
            move_group.stop()

    # kill subprocesses
    def kill(self):
        if self.killer == "init":
            self.init_mot_kill()
            self.killer = None

        elif self.killer == "rotate":
            self.rotate_mot_kill()
            self.killer = None

        elif self.killer == "nostr":
            self.init_nostr_kill()
            self.killer = None

        elif self.killer == "control":
            self.con_rob_kill()
            self.killer = None

        else:
            pass


class SwitchNode(object):
    def __init__(self):
        self.switch = Switch()

        rospy.init_node("switch", anonymous=True)

        self.subscriber = rospy.Subscriber(
            "/switch", String, self.callback, queue_size=10
        )

        print("----------------------------------------------------------")
        print("               STARTING PROCESS SWITCH                    ")
        print("----------------------------------------------------------")
        print("[INFO] Switch of ROS-bridge is connected and running!")

        rospy.spin()

    def callback(self, data):
        if data.data == "disconnect":
            self.switch.disconnect()

        elif data.data == "emergency_stop":
            self.switch.emergency_stop()

        elif data.data == "manual_control":
            self.switch.kill()
            self.switch.con_rob()

        elif data.data == "manual_control_stop":
            self.switch.con_rob_kill()

        elif data.data == "init_mot":
            self.switch.kill()
            self.switch.init_mot()

        elif data.data == "rotate_mot":
            self.switch.kill()
            self.switch.rotate_mot()

        elif data.data == "nostrill":
            self.switch.kill()
            self.switch.init_nostr()

        elif data.data == "rg2_open":
            open_rg2()

        elif data.data == "rg2_close":
            close_rg2()
        
        elif data.data == "all":
            self.switch.kill()
            self.switch.disconnect()

        else:
            self.switch.connect(data.data)


if __name__ == "__main__":
    while not rospy.is_shutdown():
        node = SwitchNode()
