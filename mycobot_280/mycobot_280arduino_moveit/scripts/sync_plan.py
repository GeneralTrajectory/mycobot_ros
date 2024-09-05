#!/usr/bin/env python2
import math
import time
import rospy
from sensor_msgs.msg import JointState

from pymycobot.mycobot import MyCobot


mc = None


def callback(data):
    global latest_data
    latest_data = [round(math.degrees(value), 2) for value in data.position]
    rospy.loginfo(f"Joint angles: {latest_data}")

def control_loop(event):
    if latest_data:
        rospy.loginfo(f"Sending angles: {latest_data}")
        mc.send_angles(latest_data, 25)

def listener():
    global mc
    rospy.init_node("control_slider", anonymous=True)

    rospy.Subscriber("joint_states", JointState, callback)
    port = rospy.get_param("~port", "/dev/ttyUSB0")
    baud = rospy.get_param("~baud", 115200)
    print(port, baud)
    mc = MyCobot(port, baud)
    time.sleep(2) # open port,need wait

    # 启动定时器，每0.5秒执行一次控制循环
    rospy.Timer(rospy.Duration(0.5), control_loop)

    rospy.spin()


if __name__ == "__main__":
    listener()
