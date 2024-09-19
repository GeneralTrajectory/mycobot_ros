#!/usr/bin/env python3

"""[summary]
This file obtains the joint angle of the manipulator in ROS,
and then sends it directly to the real manipulator using `pymycobot` API.
This file is [slider_control.launch] related script.
Passable parameters:
    port: serial prot string. Defaults is '/dev/ttyUSB0'
    baud: serial prot baudrate. Defaults is 115200.
"""
import time
import math
import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Header

from pymycobot.myarm import MyArm

def main():
    rospy.init_node("control_slider", anonymous=True)

    # Parameters
    port = rospy.get_param("~port", "/dev/ttyUSB0")
    baud = rospy.get_param("~baud", "115200")
    simulation = rospy.get_param("~simulation", True)

    if not simulation:
        print("Connecting to physical robot...")
        # Initialize connection to the physical robot
        mc = MyArm(port=port, baudrate=baud, timeout=0.1, debug=False)
        time.sleep(0.05)
        mc.set_fresh_mode(1)
        time.sleep(0.05)

        # Here, implement the logic to send joint commands to the robot
        # For example, you could read desired joint positions from a topic or generate them programmatically
    else:
        print("Running in simulation mode.")
        # Initialize joint state publisher
        joint_state_pub = rospy.Publisher('joint_states', JointState, queue_size=10)
        rate = rospy.Rate(10)  # Publish at 10 Hz

        # Joint names (must match your URDF)
        joint_names = ["joint1_to_base", "joint2_to_joint1", "joint3_to_joint2", "joint4_to_joint3", "joint5_to_joint4", "joint6_to_joint5"]

        # Main loop
        while not rospy.is_shutdown():
            # Generate joint positions programmatically
            current_time = rospy.get_time()
            joint_positions = [
                math.sin(current_time) * math.radians(30),
                math.sin(current_time) * math.radians(45),
                math.sin(current_time) * math.radians(60),
                math.sin(current_time) * math.radians(30),
                math.sin(current_time) * math.radians(45),
                math.sin(current_time) * math.radians(60)
            ]

            # Create JointState message
            joint_state = JointState()
            joint_state.header = Header()
            joint_state.header.stamp = rospy.Time.now()
            joint_state.name = joint_names
            joint_state.position = joint_positions

            # Publish the joint state
            joint_state_pub.publish(joint_state)

            # Log the joint positions
            rospy.loginfo("Published joint positions: %s", joint_positions)

            rate.sleep()

if __name__ == "__main__":
    main()