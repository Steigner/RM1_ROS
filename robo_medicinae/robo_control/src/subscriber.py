#!/usr/bin/env python

# library -> pure python client library for ROS
import rospy

# library -> Moveit python functions for ROS
import moveit_commander

# library -> geometry_msgs provides messages for common geometric primitives types
from geometry_msgs.msg import WrenchStamped

# library -> message types representing primitive data types
from std_msgs.msg import String


class Stop(object):
    # subscribe data from HEX-E sensor
    def __init__(self):
        self.move_group = moveit_commander.MoveGroupCommander("manipulator")
        rospy.init_node("HEX", anonymous=True)
        rospy.Subscriber("HEXSIM", WrenchStamped, self.callback)
        rospy.spin()

    def callback(self, data):
        wrench = WrenchStamped()
        wrench.header.frame_id = data.header.frame_id
        wrench.wrench.force.x = data.wrench.force.x

        # if Force is less than 5N STOP!
        if data.wrench.force.z < -5:
            for i in range(20):
                self.move_group.stop()
            
            pub = rospy.Publisher("info", String, queue_size=10)

            # 10 Hz
            r = rospy.Rate(10)

            for i in range(2):
                pub.publish("emergency")
                r.sleep()

if __name__ == "__main__":
    stop = Stop()