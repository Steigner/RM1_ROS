#! /usr/bin/env python2

# source: https://github.com/kbogert/UR3e_RG2_ER5/blob/master/UR3e_RG2_ER5_gripper_control/src/RG2Grip.py

# library -> a pure Python client library for ROS
import rospy

# library -> message types representing primitive data types
from std_msgs.msg import String, Float64

# library -> python socket communication
import socket

# library -> python codecs registry
from codecs import encode

# store ip adress of robot
class IP(object):
    ip = "127.0.0.1"

    def __init__(self):
        pass


class RG2(object):
    def __init__(self):
        # rospy.init_node('gripper_controller', anonymous=True)

        self.pub = rospy.Publisher(
            "/ur_hardware_interface/script_command", String, queue_size=10
        )
        self.sub = rospy.Subscriber(
            "/gripper_controller/command", Float64, self.callback
        )

    def gripper_action_controller(self, prog):
        self.pub.publish(prog)

    def callback(self, data):
        width_data = data.data
        self.setWidth(width_data)

    def play(self):
        HOST = IP.ip
        PORT = 29999

        if HOST != "127.0.0.1":
            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            s.connect((HOST, PORT))
            post = encode("play" + "\n")
            s.send(post)
            s.close()

    def setWidth(self, width, force=20):
        progrg2 = ""
        if width >= 0 and width <= 110 and force >= 3 and force <= 40:
            progrg2 = "def rg2grpCntrl():\n"
            progrg2 += '	textmsg("inside RG2 function called")\n'
            progrg2 += "	target_width=" + str(width) + "\n"
            progrg2 += "	target_force=" + str(force) + "\n"
            progrg2 += "	timeoutLength=400\n"
            progrg2 += "	timeoutLengthSecondary=20\n"
            progrg2 += "	payload=1.0\n"
            progrg2 += "	set_payload1=False\n"
            progrg2 += "	depth_compensation=False\n"
            progrg2 += "	slave=False\n"
            progrg2 += "	timeout = 0\n"
            progrg2 += "	while get_digital_in(9) == False:\n"
            progrg2 += '		textmsg("inside while")\n'
            progrg2 += "		if timeout > timeoutLength:\n"
            progrg2 += "			break\n"
            progrg2 += "		end\n"
            progrg2 += "		timeout = timeout+1\n"
            progrg2 += "		sync()\n"
            progrg2 += "	end\n"
            progrg2 += '	textmsg("outside while")\n'
            progrg2 += "	def bit(input):\n"
            progrg2 += "		msb=65536\n"
            progrg2 += "		local i=0\n"
            progrg2 += "		local output=0\n"
            progrg2 += "		while i<17:\n"
            progrg2 += "			set_digital_out(8,True)\n"
            progrg2 += "			if input>=msb:\n"
            progrg2 += "				input=input-msb\n"
            progrg2 += "				set_digital_out(9,False)\n"
            progrg2 += "			else:\n"
            progrg2 += "				set_digital_out(9,True)\n"
            progrg2 += "			end\n"
            progrg2 += "			if get_digital_in(8):\n"
            progrg2 += "				out=1\n"
            progrg2 += "			end\n"
            progrg2 += "			sync()\n"
            progrg2 += "			set_digital_out(8,False)\n"
            progrg2 += "			sync()\n"
            progrg2 += "			input=input*2\n"
            progrg2 += "			output=output*2\n"
            progrg2 += "			i=i+1\n"
            progrg2 += "		end\n"
            progrg2 += "		return output\n"
            progrg2 += "	end\n"
            progrg2 += '	textmsg("outside bit definition")\n'
            progrg2 += "	target_width=target_width+0.0\n"
            progrg2 += "	if target_force>40:\n"
            progrg2 += "		target_force=40\n"
            progrg2 += "	end\n"
            progrg2 += "	if target_force<4:\n"
            progrg2 += "		target_force=4\n"
            progrg2 += "	end\n"
            progrg2 += "	if target_width>110:\n"
            progrg2 += "		target_width=110\n"
            progrg2 += "	end\n"
            progrg2 += "	if target_width<0:\n"
            progrg2 += "		target_width=0\n"
            progrg2 += "	end\n"
            progrg2 += "	rg_data=floor(target_width)*4\n"
            progrg2 += "	rg_data=rg_data+floor(target_force/2)*4*111\n"
            progrg2 += "	if slave:\n"
            progrg2 += "		rg_data=rg_data+16384\n"
            progrg2 += "	end\n"
            progrg2 += '	textmsg("about to call bit")\n'
            progrg2 += "	bit(rg_data)\n"
            progrg2 += '	textmsg("called bit")\n'
            progrg2 += "	if depth_compensation:\n"
            progrg2 += "		finger_length = 55.0/1000\n"
            progrg2 += "		finger_heigth_disp = 5.0/1000\n"
            progrg2 += "		center_displacement = 7.5/1000\n"
            progrg2 += "		start_pose = get_forward_kin()\n"
            progrg2 += "		set_analog_inputrange(2, 1)\n"
            progrg2 += "		zscale = (get_analog_in(2)-0.026)/2.976\n"
            progrg2 += "		zangle = zscale*1.57079633-0.087266462\n"
            progrg2 += "		zwidth = 5+110*sin(zangle)\n"
            progrg2 += "		start_depth = cos(zangle)*finger_length\n"
            progrg2 += "		sync()\n"
            progrg2 += "		sync()\n"
            progrg2 += "		timeout = 0\n"
            progrg2 += "		while get_digital_in(9) == True:\n"
            progrg2 += "			timeout=timeout+1\n"
            progrg2 += "			sync()\n"
            progrg2 += "			if timeout > timeoutLengthSecondary:\n"
            progrg2 += "				break\n"
            progrg2 += "			end\n"
            progrg2 += "		end\n"
            progrg2 += "		timeout = 0\n"
            progrg2 += "		while get_digital_in(9) == False:\n"
            progrg2 += "			zscale = (get_analog_in(2)-0.026)/2.976\n"
            progrg2 += "			zangle = zscale*1.57079633-0.087266462\n"
            progrg2 += "			zwidth = 5+110*sin(zangle)\n"
            progrg2 += "			measure_depth = cos(zangle)*finger_length\n"
            progrg2 += "			compensation_depth = (measure_depth - start_depth)\n"
            progrg2 += "			target_pose = pose_trans(start_pose,p[0,0,-compensation_depth,0,0,0])\n"
            progrg2 += "			if timeout > timeoutLength:\n"
            progrg2 += "				break\n"
            progrg2 += "			end\n"
            progrg2 += "			timeout=timeout+1\n"
            progrg2 += "			servoj(get_inverse_kin(target_pose),0,0,0.008,0.033,1700)\n"
            progrg2 += "		end\n"
            progrg2 += "		nspeed = norm(get_actual_tcp_speed())\n"
            progrg2 += "		while nspeed > 0.001:\n"
            progrg2 += "			servoj(get_inverse_kin(target_pose),0,0,0.008,0.033,1700)\n"
            progrg2 += "			nspeed = norm(get_actual_tcp_speed())\n"
            progrg2 += "		end\n"
            progrg2 += "	end\n"
            progrg2 += "	if depth_compensation==False:\n"
            progrg2 += "		timeout = 0\n"
            progrg2 += "		while get_digital_in(9) == True:\n"
            progrg2 += "			timeout = timeout+1\n"
            progrg2 += "			sync()\n"
            progrg2 += "			if timeout > timeoutLengthSecondary:\n"
            progrg2 += "				break\n"
            progrg2 += "			end\n"
            progrg2 += "		end\n"
            progrg2 += "		timeout = 0\n"
            progrg2 += "		while get_digital_in(9) == False:\n"
            progrg2 += "			timeout = timeout+1\n"
            progrg2 += "			sync()\n"
            progrg2 += "			if timeout > timeoutLength:\n"
            progrg2 += "				break\n"
            progrg2 += "			end\n"
            progrg2 += "		end\n"
            progrg2 += "	end\n"
            progrg2 += "	if set_payload1:\n"
            progrg2 += "		if slave:\n"
            progrg2 += "			if get_analog_in(3) < 2:\n"
            progrg2 += "				zslam=0\n"
            progrg2 += "			else:\n"
            progrg2 += "				zslam=payload\n"
            progrg2 += "			end\n"
            progrg2 += "		else:\n"
            progrg2 += "			if get_digital_in(8) == False:\n"
            progrg2 += "				zmasm=0\n"
            progrg2 += "			else:\n"
            progrg2 += "				zmasm=payload\n"
            progrg2 += "			end\n"
            progrg2 += "		end\n"
            progrg2 += "		zsysm=0.0\n"
            progrg2 += "		zload=zmasm+zslam+zsysm\n"
            progrg2 += "		set_payload(zload)\n"
            progrg2 += "	end\n"
            progrg2 += "end\n"
            progrg2 += "rg2grpCntrl()\n"
            self.gripper_action_controller(progrg2)


def open_rg2():
    rg2 = RG2()
    rospy.sleep(2)
    rg2.setWidth(50)
    print("[INFO] Gripper - open")
    rospy.sleep(1)
    rg2.play()
    print("[INFO] Gripper - UR play")


def close_rg2():
    rg2 = RG2()
    rospy.sleep(2)
    rg2.setWidth(28)
    print("[INFO] Gripper - close")
    rospy.sleep(1)
    rg2.play()
    print("[INFO] Gripper - UR play")
