#!/usr/bin/env python2

# library -> system-specific parameters and functions
import sys

# library -> pure python client library for ROS
import rospy

# library -> Moveit python functions for ROS
import moveit_commander
from moveit_commander.conversions import pose_to_list

# library -> geometry_msgs provides messages for common geometric primitives types
import geometry_msgs

# library -> shallow and deep copy operations
import copy

# library -> built-in math functions, including an extensive math module
import math

# library -> tf maintains the relationship between coordinate frames in a tree structure
import tf

# library -> message types representing primitive data types
from std_msgs.msg import String

# script  -> open, close RG2 gripper
from rg2 import open_rg2, close_rg2


def all_close(goal, actual, tolerance):
    all_equal = True
    if type(goal) is list:
        for index in range(len(goal)):
            if abs(actual[index] - goal[index]) > tolerance:
                return False

    elif type(goal) is geometry_msgs.msg.PoseStamped:
        return all_close(goal.pose, actual.pose, tolerance)

    elif type(goal) is geometry_msgs.msg.Pose:
        return all_close(pose_to_list(goal), pose_to_list(actual), tolerance)

    return True


class Robot_Motion(object):
    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)

        rospy.init_node("TESTOS", anonymous=True)

        robot = moveit_commander.RobotCommander()
        scene = moveit_commander.PlanningSceneInterface()
        move_group = moveit_commander.MoveGroupCommander("manipulator")

        self.box_name = ""
        self.scene = scene
        self.robot = robot

        self.move_group = move_group

        self.trans = None
        self.rot = None

        self.sub = None

    # lookupTransform function
    # https://answers.ros.org/question/202130/stampedtransform-getting-position-instead-of-relative-position/
    def get_tf(self, link):
        listener = tf.TransformListener()

        rate = rospy.Rate(10.0)

        while not rospy.is_shutdown():
            try:
                (trans, rot) = listener.lookupTransform(
                    link, "/camera_link", rospy.Time(0)
                )

                self.trans = trans
                self.rot = rot

                rospy.sleep(1)
                return 0

            except (
                tf.LookupException,
                tf.ConnectivityException,
                tf.ExtrapolationException,
            ):
                continue

    # CIRCLE MOTION ATTEMPT ----------------------------------------------------------------------------------
    def pos_test(self):
        move_group = self.move_group
        waypoints = []
        wpose = move_group.get_current_pose().pose
        y_center = wpose.position.y
        z_center = wpose.position.z

        # numpy linspace alternative
        size = 360
        from_ = 0
        to_ = 2 * math.pi
        theta = [from_ + x * (to_ - from_) / (size - 1) for x in range(size)]
        r = 0.001

        for i in range(size):
            wpose.position.y = y_center + r * math.cos(theta[i])
            wpose.position.z = z_center + r * math.sin(theta[i])

            waypoints.append(copy.deepcopy(wpose))

        (plan, fraction) = move_group.compute_cartesian_path(
            waypoints, 0.0001, 0
        )

        move_group.execute(plan, wait=True)
        move_group.stop()

    # --------------------------------------------------------------------------------------------------------

    def pos_stick_test1(self, pos):
        if pos == "stick":
            x = -0.66
            y = -0.689 + 0.0275
            z = 1.18126482214 + 0.17

        else:
            x = -0.717
            y = -0.689
            z = 1.18126482214 + 0.17

        move_group = self.move_group
        pose = move_group.get_current_pose().pose

        pose_goal = geometry_msgs.msg.Pose()
        pose_goal.position.x = x
        pose_goal.position.y = y
        pose_goal.position.z = z
        pose_goal.orientation.x = pose.orientation.x
        pose_goal.orientation.y = pose.orientation.y
        pose_goal.orientation.z = pose.orientation.z
        pose_goal.orientation.w = pose.orientation.w

        move_group.set_pose_target(pose_goal)
        plan = move_group.go(wait=True)

        move_group.stop()

        move_group.clear_pose_targets()

        current_pose = self.move_group.get_current_pose().pose
        return all_close(pose_goal, current_pose, 0.01)

    def pos_go_updown(self, mot):
        if mot == "up":
            direct = 0.16

        elif mot == "drop":
            direct = -0.13

        else:
            direct = -0.2

        move_group = self.move_group

        waypoints = []

        wpose = move_group.get_current_pose().pose
        wpose.position.z = wpose.position.z + direct

        waypoints.append(copy.deepcopy(wpose))

        (plan, fraction) = move_group.compute_cartesian_path(
            waypoints, 0.01, 0.0
        )

        move_group.execute(plan, wait=True)
        move_group.stop()

    def init_motion(self):
        move_group = self.move_group

        move_group.allow_replanning(True)
        move_group.allow_looking(True)

        joint_goal = move_group.get_current_joint_values()

        # position pred spilky
        joint_goal[0] = math.radians(-5)
        joint_goal[1] = math.radians(-115)
        joint_goal[2] = math.radians(70)
        joint_goal[3] = math.radians(315)
        joint_goal[4] = math.radians(270)
        joint_goal[5] = math.radians(175)
        move_group.go(joint_goal, wait=True)
        move_group.stop()

        current_joints = move_group.get_current_joint_values()
        return all_close(joint_goal, current_joints, 0.01)

    def pos_scan(self):
        move_group = self.move_group

        move_group.allow_replanning(True)
        move_group.allow_looking(True)

        joint_goal = move_group.get_current_joint_values()

        joint_goal[0] = math.radians(118)
        joint_goal[1] = math.radians(-15)
        joint_goal[2] = math.radians(-53)
        joint_goal[3] = math.radians(230)
        joint_goal[4] = math.radians(64)
        joint_goal[5] = math.radians(189)
        move_group.go(joint_goal, wait=True)

        move_group.stop()
        current_joints = move_group.get_current_joint_values()
        return all_close(joint_goal, current_joints, 0.01)

    def pos_0(self):
        move_group = self.move_group
        move_group.allow_replanning(True)
        move_group.allow_looking(True)
        joint_goal = move_group.get_current_joint_values()
        joint_goal[0] = math.radians(-5)
        joint_goal[1] = math.radians(-92)
        joint_goal[2] = math.radians(17)
        joint_goal[3] = math.radians(345)
        joint_goal[4] = math.radians(270)
        joint_goal[5] = math.radians(175)
        move_group.go(joint_goal, wait=True)

        move_group.stop()
        current_joints = move_group.get_current_joint_values()
        return all_close(joint_goal, current_joints, 0.01)

    def pos_rotate(self):
        def go(data):

            move_group = self.move_group

            pose = move_group.get_current_pose().pose
            orint = [
                pose.orientation.x,
                pose.orientation.y,
                pose.orientation.z,
                pose.orientation.w,
            ]
            [r, p, y] = tf.transformations.euler_from_quaternion(orint)

            if data.data == "up":
                if math.degrees(r) > 76:
                    r = math.radians(math.degrees(r) - 5)

                else:
                    pass

            elif data.data == "stop":
                self.sub.unregister()

            elif data.data == "down":
                if math.degrees(r) < 126:
                    r = math.radians(math.degrees(r) + 5)

                else:
                    pass

            else:
                pass

            # print(math.degrees(r),math.degrees(p),math.degrees(y))

            [x, y, z, w] = tf.transformations.quaternion_from_euler(r, p, y)

            pose_goal = geometry_msgs.msg.Pose()
            pose_goal.position.x = pose.position.x
            pose_goal.position.y = pose.position.y
            pose_goal.position.z = pose.position.z
            pose_goal.orientation.x = x
            pose_goal.orientation.y = y
            pose_goal.orientation.z = z
            pose_goal.orientation.w = w

            move_group.set_pose_target(pose_goal)
            move_group.go(wait=True)
            move_group.stop()

        while not rospy.is_shutdown():
            self.sub = rospy.Subscriber("up_down", String, go)
            rospy.spin()

    def add_plane(self, timeout=4):
        box_name = self.box_name
        scene = self.scene

        box_pose = geometry_msgs.msg.PoseStamped()
        box_pose.header.frame_id = "world"

        box_pose.pose.position.x = -1.1
        box_pose.pose.position.y = -0.5
        box_pose.pose.position.z = 1

        box_name = "plane"

        scene.add_box(box_name, box_pose, size=(0.1, 2, 2))
        self.box_name = box_name

        return self.wait_for_state_update(box_is_known=True, timeout=timeout)

    def add_stick(self, timeout=4):
        box_name = self.box_name
        scene = self.scene

        box_pose = geometry_msgs.msg.PoseStamped()
        box_pose.header.frame_id = "world"

        box_pose.pose.position.x = -0.6605
        box_pose.pose.position.y = -0.689 + 0.0275
        box_pose.pose.position.z = 0.8865

        box_name = "stick"

        scene.add_cylinder(box_name, box_pose, 0.098, 0.001)
        self.box_name = box_name

        return self.wait_for_state_update(box_is_known=True, timeout=timeout)

    def attach_box(self, timeout=4):
        box_name = self.box_name
        robot = self.robot
        scene = self.scene
        eef_link = self.move_group.get_end_effector_link()

        grasping_group = "end_effector"
        touch_links = robot.get_link_names(group=grasping_group)
        scene.attach_box(eef_link, box_name, touch_links=touch_links)

        return self.wait_for_state_update(
            box_is_attached=True, box_is_known=False, timeout=timeout
        )

    def detach_box(self, timeout=4):
        box_name = "stick"
        scene = self.scene
        eef_link = self.move_group.get_end_effector_link()

        scene.remove_attached_object(eef_link, name=box_name)

        return self.wait_for_state_update(
            box_is_known=True, box_is_attached=False, timeout=timeout
        )

    def wait_for_state_update(
        self, box_is_known=False, box_is_attached=False, timeout=4
    ):
        box_name = self.box_name
        scene = self.scene
        start = rospy.get_time()
        seconds = rospy.get_time()
        while (seconds - start < timeout) and not rospy.is_shutdown():
            attached_objects = scene.get_attached_objects([box_name])
            is_attached = len(attached_objects.keys()) > 0
            is_known = box_name in scene.get_known_object_names()
            if (box_is_attached == is_attached) and (box_is_known == is_known):
                return True
        rospy.sleep(0.1)
        seconds = rospy.get_time()
        return False

    def remove_box(self, timeout=4):
        scene = self.scene
        scene.remove_world_object("plane")
        # scene.remove_world_object("stick")
        return self.wait_for_state_update(
            box_is_attached=False, box_is_known=False, timeout=timeout
        )

    def pub(self):
        pub = rospy.Publisher("info", String, queue_size=10)

        r = rospy.Rate(10)  # 10hz

        for i in range(20):
            pub.publish("done")
            r.sleep()

    def pos_align(self):
        move_group = self.move_group
        pose = move_group.get_current_pose().pose

        t1 = pose.orientation.x
        t2 = pose.orientation.y
        t3 = pose.orientation.z
        t4 = pose.orientation.w

        # auto align
        [x, y, z] = tf.transformations.euler_from_quaternion([t1, t2, t3, t4])
        # [t1,t2,t3,t4] = tf.transformations.quaternion_from_euler(math.radians(28.91999166117532 + 20), y, z )

        [t1, t2, t3, t4] = tf.transformations.quaternion_from_euler(
            x - math.radians(40), y, z
        )

        pose_goal = geometry_msgs.msg.Pose()
        pose_goal.position.x = pose.position.x - self.trans[2]
        pose_goal.position.y = pose.position.y - self.trans[0]
        pose_goal.position.z = pose.position.z + self.trans[1]
        pose_goal.orientation.x = t1
        pose_goal.orientation.y = t2
        pose_goal.orientation.z = t3
        pose_goal.orientation.w = t4

        move_group.set_pose_target(pose_goal)
        plan = move_group.go(wait=True)
        move_group.stop()

    def pos_align_move(self, point):
        # In robot system represented as [y,z,x]
        # test = [-0.0011257142680031914, -0.040150475558780485, 0.39399999380111694]
        # test = [ 0.00324, -0.02268,  0.486 ]
        # test = [0.00121143, -0.03270857,  0.42399999]

        move_group = self.move_group

        waypoints = []

        # 0.01 -> static due to z distance from edge of camera
        # 0.0325 -> static due to y distance from edge of camera
        wpose = move_group.get_current_pose().pose
        wpose.position.z = wpose.position.z - 0.0125 - point[1]
        wpose.position.y = wpose.position.y + point[0] - 0.00375

        waypoints.append(copy.deepcopy(wpose))

        (plan, fraction) = move_group.compute_cartesian_path(
            waypoints, 0.01, 0.0 
        )
        move_group.execute(plan, wait=True)

    def pos_move_up(self, point):
        # len_stick = 0.08
        len_stick = 0.098 - 0.0125

        move_group = self.move_group

        t = move_group.get_current_rpy()
        # print(math.degrees(t[0]), math.degrees(t[1]), math.degrees(t[2]))

        waypoints = []
        wpose = move_group.get_current_pose().pose
        wpose.position.z = wpose.position.z + (
            # math.tan((math.pi / 2) - t[0]) * (point[2] - 0.299)
            math.tan((math.pi / 2) - t[0]) * (point[2] - (0.247 + 0.0125) - len_stick)
        )
        wpose.position.x = wpose.position.x - (point[2] - (0.247 + 0.0125) - len_stick)
        waypoints.append(copy.deepcopy(wpose))

        (plan, fraction) = move_group.compute_cartesian_path(
            waypoints, 0.01, 0.0 
        )

        move_group.execute(plan, wait=True)

    def pos_move_down(self, point):
        move_group = self.move_group
        t = move_group.get_current_rpy()

        waypoints = []
        wpose = move_group.get_current_pose().pose
        wpose.position.z = wpose.position.z - (
            math.tan((math.pi / 2) - t[0]) * (point[2] - 0.299)
        )
        wpose.position.x = wpose.position.x + (point[2] - 0.299)
        waypoints.append(copy.deepcopy(wpose))

        (plan, fraction) = move_group.compute_cartesian_path(
            waypoints, 0.01, 0.0
        )

        move_group.execute(plan, wait=True)


def init_motion():
    try:
        print("----------------------------------------------------------")
        print("               STARTING PROCESS: INIT                     ")
        print("----------------------------------------------------------")

        mot = Robot_Motion()
        # add stick and safety barrier
        rospy.sleep(2)
        mot.add_plane()
        mot.add_stick()

        print("[INFO] Added into simulation stick and safety plane!")

        # init position
        mot.pos_0()

        print("[INFO] Reached position INIT!")

        # mot to coordinates to 1. stick
        mot.pos_stick_test1("stick")

        print("[INFO] Reached position STICK1")

        # open gripper -> rg2.py
        open_rg2()

        print("[INFO] Gripper opened!")

        # car. motion down
        mot.pos_go_updown("down")

        print("[INFO] Reached position DOWN")

        mot.attach_box()

        # close gripper -> rg2.py
        close_rg2()

        print("[INFO] Gripper closed!")

        # car. motion up
        mot.pos_go_updown("up")

        print("[INFO] Reached position UP!")

        # position scan
        mot.pos_scan()

        print("[INFO] Reached position SCAN!")

        mot.pub()

    except rospy.ROSInterruptException:
        return


def rotate_motion():
    try:
        print("----------------------------------------------------------")
        print("           STARTING PROCESS: ROTATE                       ")
        print("----------------------------------------------------------")
        mot = Robot_Motion()
        # mot.pos_scan()
        rospy.sleep(2)
        mot.remove_box()
        # mot.pos_rotate()
        mot.pos_scan()

    except rospy.ROSInterruptException:
        return


def nostrill_init():
    try:
        print("----------------------------------------------------------")
        print("           STARTING PROCESS: TAKE SAMPLE                  ")
        print("----------------------------------------------------------")

        mot = Robot_Motion()

        rospy.sleep(1)

        print("[INFO] Waiting for point!")
        point = rospy.get_param('point')

        # TEST PURPOSE:
        # point = [0.025268396, 0.085912548, 0.50927734]
        # point = [-0.006242857234818595, -0.05785047704265231, 0.43700000643730164]
        print("[INFO] Get point: " + str(point))

        # for testing process
        mot.pos_scan()

        print("[INFO] Align process!")
        mot.get_tf("/tool0")
        # align in x,y,z and rotate 40 degrees
        mot.pos_align()

        mot.pos_align_move(point)

        mot.pos_move_up(point)
        print("[INFO] Reached position nostril!")

        print("[INFO] Circular motion!")
        mot.pos_test()

        mot.pos_move_down(point)
        mot.pos_scan()
        print("[INFO] Reached position SCAN!")

        mot.pos_0()
        print("[INFO] Reached position INIT!")

        mot.pos_stick_test1("test")
        print("[INFO] Reached position drop STICK1")

        mot.pos_go_updown("drop")

        print("[INFO] Reached position to DROP")

        rospy.sleep(0.5)
        mot.detach_box()
        rospy.sleep(0.5)

        open_rg2()
        
        print("[INFO] Gripper opened!")
        
        rospy.sleep(2)

        mot.pub()

    except rospy.ROSInterruptException:
        return


if __name__ == "__main__":
    switch = int(sys.argv[1])

    if switch == 1:
        init_motion()

    elif switch == 2:
        rotate_motion()

    elif switch == 3:
        nostrill_init()

    else:
        pass
