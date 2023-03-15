#!/usr/bin/env python

"""
#title           :HMC5983.py
#description     :Python Script to read HMC5983 Geomagnetic sensor via I2C communication
#author          :  - Achmad Syahrul Irwansyah (ach.syahrul99@gmail.com)
                    - M. Luthfi Hariyadin
#date            :2023/02/28
#version         :1.0
#usage           :Python
#notes           :Haven't tested yet
#==============================================================================
"""

from smbus import SMBus
import time
import math
import rospy
from std_msgs.msg import Float32

#define register list from datasheet
CONRA = 0x00 #Configuration Register A
CONRB = 0x01 #Configuration Register B
MODREG = 0x02 #Mode Register
DOXMSB = 0x03 #Data Output X MSB Register
DOXLSB = 0x04 #Data Output X LSB Register
DOZMSB = 0x05 #Data Output Z MSB Register
DOZLSB = 0x06 #Data Output Z LSB Register
DOYMSB = 0x07 #Data Output Y MSB Register
DOYLSB = 0x08 #Data Output Y LSB Register
STAREG = 0x09 #Status Register
IDREGA = 0x0A #Identification Register A
IDREGB = 0x0B #Identification Register B
IDREGC = 0x0C #Identification Register C
TEMPMSB = 0x31 #Temperature Output MSB Register
TEMPLSB = 0x32 #Temperature Output LSB Register

"""
From datasheet for the HMC5983
Below is an example of a (power-on) initialization process for “continuous-measurement mode” via I2C interface:
1. Write CRA (00) - send 0x3C 0x00 0x70 (8-average, 15 Hz default or any other rate, normal measurement)
2. Write CRB (01) - send 0x3C 0x01 0xA0 (Gain=5, or any other desired gain)
3. For each measurement query:
	Write Mode (02) - send 0x3C 0x02 0x01 (Single-measurement mode)
	Wait 6 ms or monitor status register or DRDY hardware interrupt pin
	Send 0x3D 0x06 (Read all 6 bytes. If gain is changed then this data set is using previous gain)
	Convert three 16-bit 2's compliment hex values to decimal values and assign to X, Z, Y, respectively.

Please refer to the datasheet for the detail information: 
https://www.farnell.com/datasheets/1509871.pdf?_ga=2.219060057.1318745487.1555987311-293789508.1555987311

I2C with Python code reference:
https://www.abelectronics.co.uk/kb/article/1094/i2c-part-4---programming-i-c-with-python
"""

def read_HMC ():
    HMC5983_ADDR = 0x1E # The default address of HMC5893
    HMC5983 = SMBus(1) # Create a new I2C bus (please do 'ls /dev/*i2c*' to detect the bus number)

    HMC5983.write_byte_data(HMC5983_ADDR, CONRA, 0x10) #Conf. Reg. A (00010000): Temperature sensor is disable, 1 sample average per measurement, 15 Hz data output, Normal measurement configuration
    HMC5983.write_byte_data(HMC5983_ADDR, CONRB, 0x20) #Conf. Reg. B (00100000): Sensor Field Range of ±1.3 Ga with 0.92 mG/LSb digital resolution (default)
    HMC5983.write_byte_data(HMC5983_ADDR, MODREG, 0x01) #Mode Register (00000001): Single-Measurement Mode (default)

    block = HMC5983.read_i2c_block_data(HMC5983_ADDR, DOXMSB, 32)

    x_msb = block[0]
    x_lsb = block[1]
    z_msb = block[2]
    z_lsb = block[3]
    y_msb = block[4]
    y_lsb = block[5]

    x_raw = x_msb * 256 + x_lsb
    z_raw = z_msb * 256 + z_lsb
    y_raw = y_msb * 256 + y_lsb

    x = confit(x_raw)
    z = confit(z_raw)
    y = confit(y_raw)

    # initializing the publisher node
    rospy.init_node('IMU_node', anonymous=True)

    # Create a publisher
    HMC_pub = rospy.Publisher('heading_data', Float32, queue_size=10)

    # Create the message
    heading = Float32()

    # set the rate at which values will be published 
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        heading.data = math.atan2(y, x) * 180 / math.pi

        # Publish the message
        rospy.loginfo("Heading: %f", heading.data)
        HMC_pub.publish(heading)
        rate.sleep()
        
def confit(value):
    if value > 32768:
        return value - 65535
    else:
        return value

if __name__ == "__main__":
    try:
        read_HMC()
    except rospy.ROSInterruptException:
        pass
    

