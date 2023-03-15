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
from geometry_msgs.msg import Pose
from std_msgs.msg import Float32
import serial
import pynmea2 #library for parsing GPS NMEA format

heading = -999
def callback_heading(heading_HMC):
    # Assign heading value from the subscribed message
    global heading
    heading = heading_HMC.data

def listen_heading():
    # listen heading from IMU_node
    while not rospy.is_shutdown():
        rospy.Subscriber('heading_data', Float32, callback_heading)

def pub_gps():
    global heading
    cond = "trouble: init. USB"
    # setup serial connection
    try:
        gps=serial.Serial("/dev/ttyUSB0",9600,timeout=1)
        gps.baudrate=9600
        gps.reset_input_buffer()
    except:
        print(cond)

    # Create a publisher which can "talk" to Turtlesim and tell it to move
    pose_gps = rospy.Publisher('pose_gps', Pose, queue_size=10)

    # Create the message
    gps_msg = Pose()

    # initializing the publisher node
    rospy.init_node('gps_node', anonymous=True)

    # set the rate at which values will be published 
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        cond = "reading serial"
        try:
            # read serial line and decode it into variables
            line_gps=gps.readline().decode('utf-8', errors='replace').strip()
            if '$GNGGA' in line_gps:
                parsed = pynmea2.parse(line_gps)
                
                # asssign the massage's value
                gps_msg.position.x = float(parsed.latitude) # left velocity
                gps_msg.position.y = float(parsed.longitude) # right velocity
                gps_msg.orientation.z = float(heading) # heading from GPS Module IMU (the yaw angle)
                print(cond)
        except:
            cond = "no GPS meas."
            print(cond)
            pass
        
        # Publish the message
        rospy.loginfo(gps_msg)
        pose_gps.publish(gps_msg)
        rate.sleep()
        
if __name__ == "__main__":
    try:
        listen_heading()
        pub_gps()
    except rospy.ROSInterruptException:
        pass