#!/usr/bin/env python
"""
#title           :ekf_node.py
#description     :Python Script for EKF algorithm in ROS environment
#author          :Nicholas Putra Rihandoko
#date            :2023/01/31
#version         :2.1
#usage           :Python
#notes           :
#python_version  :3.8
#==============================================================================
"""

import rospy
import time
from std_msgs.msg import UInt8
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose
import ekf

mode = 1
dt = 0.1
lat = 0
lon = 0
sats = 0
odo_VL = 0
odo_VR = 0

# callback function to store subscribed topics to variabele
def callback_gps(gps_msg):
    global lat, lon
    rospy.loginfo(rospy.get_caller_id())
    lat = gps_msg.position.x
    lon = gps_msg.position.y

def callback_odom(odom_msg):
    global odo_VL, odo_VR
    rospy.loginfo(rospy.get_caller_id())
    odo_VL = odom_msg.linear.x
    odo_VR = odom_msg.angular.z

def callback_mode(ekf_state_msg):
    global mode
    rospy.loginfo(rospy.get_caller_id())
    mode = ekf_state_msg


def ekf_procedure():
    global mode, dt, lat, lon, sats, odo_VL, odo_VR
    time_prev = time.time()

    # Create a new topic to publish EKF result
    pose_ekf = rospy.Publisher('pose_ekf', Pose, queue_size=10)

    # Create the message
    ekf_msg = Pose()

    # initializing the node
    rospy.init_node('ekf_node', anonymous=True)

    # set the rate at which values will be published 
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        # Subscribe topics from GPS and Arduino
        rospy.Subscriber('pose_gps', Pose, callback_gps)
        rospy.Subscriber('wheel_speed', Twist, callback_odom)
        rospy.Subscriber('ekf_state', UInt8, callback_mode)

        # calculate time step
        time_now = time.time()
        dt = time_now-time_prev
        time_prev = time_now

        # perform EKF calculation
        maps = ekf.filtering(mode,dt,lat,lon,odo_VL,odo_VR)

        # asssign the EKF's result to the message
        ekf_msg.position.x = maps[0]
        ekf_msg.position.y = maps[1]
        ekf_msg.orientation.z = maps[2]

        # Publish the message
        pose_ekf.publish(ekf_msg)
        rate.sleep()

if __name__ == "__main__":
    try:
        ekf_procedure()
    except rospy.ROSInterruptException:
        pass