#!/usr/bin/env python
import time
import serial
import rospy
from std_msgs.msg import Float64

ser = serial.Serial(
        port='/dev/ttyAMA0', #Replace ttyS0 with ttyAM0 for Pi1,Pi2,Pi0
        baudrate = 9600,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        bytesize=serial.EIGHTBITS,
        timeout=1
)

rospy.init_node("display")
#depth = rospy.wait_for_messages('/sensors/depth', Float64)
#yaw = rospy.wait_for_messages('/sensors/yaw', Float64)

while(True):
        #ser.write("Current depth: %d \n" %depth)
        #ser.write("Current yaw: %d \n" % yaw)
        ser.write("Test!")
        #time.sleep(1)
