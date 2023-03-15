#!/usr/bin/env python
"""
#title           :multiserial_pparsing.py
#description     :Python Script to read multi-serial-port + parsing to json format
#author          :Nicholas Putra Rihandoko
#date            :2022/12/12
#version         :0.1
#usage           :Python
#notes           :
#python_version  :3.8
#==============================================================================
"""

import rospy
from std_msgs.msg import UInt8
from geometry_msgs.msg import Twist
import serial

def odometry():
    cond = "trouble: init. USB"
    # setup serial connection
    try:
        odom=serial.Serial("/dev/ttyACM0",57600,timeout=1)
        odom.baudrate=57600
        odom.reset_input_buffer()
    except:
        print(cond)

    # Create a publisher which can "talk" to Turtlesim and tell it to move
    wheel_speed = rospy.Publisher('wheel_speed', Twist, queue_size=10)
    ekf_state = rospy.Publisher('ekf_state', UInt8, queue_size=10)

    # Create the message
    odom_msg = Twist()
    mode = UInt8()

    # initializing the publisher node
    rospy.init_node('arduino_node', anonymous=True)

    # set the rate at which values will be published 
    rate = rospy.Rate(20)# 10hz
    while not rospy.is_shutdown():
        cond = "reading ser2ial"
        try:
            # read serial line and decode it into variables
            line_odom=odom.readline().decode('utf-8').split(',')
            parsed = [x.rstrip() for x in line_odom]

            # asssign the massage's value
            odom_msg.linear.x = float(parsed[0]) # left velocity
            odom_msg.angular.z = float(parsed[1]) # right velocity
            mode.data = int(parsed[2])
            print(cond)
        except:
            cond = "no odometry meas."
            print(cond)
            pass
        
        # Publish the message
        #rospy.loginfo(odom_msg)
        wheel_speed.publish(odom_msg)
        ekf_state.publish(mode)
        rate.sleep()
        
if __name__ == "__main__":
    try:
        odometry()
    except rospy.ROSInterruptException:
        pass
