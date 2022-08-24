#!/usr/bin/python

import rospy
from std_msgs.msg import Float64 
import spidev
import time
import os
 
# Open SPI bus
spi = spidev.SpiDev()
spi.open(0,0)
 
# Function to read SPI data from MCP3008 chip
def ReadChannel(channel):
  adc = spi.xfer2([1,(8+channel)<<4,0])
  data = ((adc[1]&3) << 8) + adc[2]
  return data

swt_channel = 0
vrx_channel = 1
vry_channel = 2
delay = 0.5

rospy.init_node("joystick")
pubj_x = rospy.Publisher('/ESC_topic/x_axis', Float64)
pubj_y = rospy.Publisher('/ESC_topic/y_axis', Float64)
pubj_z = rospy.Publisher('/ESC_topic/z_axis', Float64)
 
while True:
 
  # Read the joystick position data
  vrx_pos = ReadChannel(vrx_channel)
  vry_pos = ReadChannel(vry_channel)
 
  # Read switch state
  swt_val = ReadChannel(swt_channel)
 
  # Print out results
  print "--------------------------------------------"
  print("X : {}  Y : {}  Switch : {}".format(vrx_pos,vry_pos,swt_val))
 
  pubj_x.publish(vrx_pos)
  pubj_y.publish(vry_pos)
  pubj_z.publish(swt_val)


  # Wait before repeating loop
  time.sleep(delay)
