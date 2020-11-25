#! /usr/bin/env python
#
# motor_controller.py
# Cory Duce cory.duce@gmail.com
# Nov 24 2020
# 
# v1.0 -Initial release
#

import rospy
import smbus
import math
from geometry_msgs.msg import Vector3
from std_msgs.msg import Bool, Float64
import RPi.GPIO as GPIO
import time
from textwrap import wrap

PIN_I2C6_POWER_ENABLE = 17

bus = smbus.SMBus(3)		#this is I2C6 on the pi4 for some reason
DEVICE_ADDRESS = 0x53
speed = 100.0
enabledState = False

def callback_speed(msg):
	global speed
	speed = msg.data
	
def callback_enable(msg):
	global enabledState
	if(msg.data == True):
		GPIO.output(PIN_I2C6_POWER_ENABLE, GPIO.HIGH)
		time.sleep(0.1)
		enabledState = True
	if(msg.data ==False):
		GPIO.output(PIN_I2C6_POWER_ENABLE, GPIO.LOW)
		enabledState = False

#takes a 32bit integer and converts it into an array of 8bit ints
def int_to_byte_array(num):
	if num < 0:
		hexStr = hex((1<<32) + num)
	else:
		hexStr = hex(num)
	hexStr = hexStr.rstrip("L")		#sometimes the hex conversion adds a 'L' to the end. Remove it
	
	padded = str.format('{:08X}',int(hexStr,16))	#pad out to 8 hex characters
	padded = wrap(padded,2)							#split into pairs of hex chars ie bytes
	
	array = [0,0,0,0]
	for x in range(4):
		array[x] = int(padded[x],16)
		
	array.reverse()		#reverse the list. We will send LSB first
	return array

#takes an array of 4 8-bit values and converts them into a 32bit integer
def byte_array_to_int(array):	
	num = array[3]*16777216 + array[2]*65536 + array[1]*256 + array[0]
	if(array[3] > 127):	#check sign bit
		num=num-4294967296
	return num

#Takes a x and y input from a joystick and mix them for differential tank style driving
def callback_drivejs(data):
	global speed
	global enabledState
	
	x = int(data.x * -speed)
	y = int(data.y * speed)

	#convert to polar
	r = math.hypot(y,x)
	t = math.atan2(x,y)

	#rotate 45 deg
	t += math.pi / 4

	#back to cartesian
	left = r * math.cos(t)
	right = r * math.sin(t)

	#scale
	left =  left * math.sqrt(2)
	right = right * math.sqrt(2)

	#constrain
	left =int( max(-1000, min(left,1000)))
	right = int(max(-1000,min(right,1000)))

	try:
		#Registers 3 and 4 hold the individual motor speed commands
		bus.write_i2c_block_data(DEVICE_ADDRESS,3,int_to_byte_array(right))
		bus.write_i2c_block_data(DEVICE_ADDRESS,4,int_to_byte_array(left))
	except:
		if( enabledState ):
			time.sleep(0.1)
			GPIO.output(PIN_I2C6_POWER_ENABLE, GPIO.LOW)	#disable the motor driver
			print("Motor I2C error")
			time.sleep(0.1)
			GPIO.output(PIN_I2C6_POWER_ENABLE, GPIO.HIGH)	#enable the motor driver

#===================================================================================

GPIO.setmode(GPIO.BCM)
GPIO.setup(PIN_I2C6_POWER_ENABLE, GPIO.OUT)

rospy.init_node('motor_controller', anonymous=True)
rospy.Subscriber("drive_joystick",Vector3, callback_drivejs)
rospy.Subscriber("motor_controller_power_enable",Bool, callback_enable)
rospy.Subscriber("motor_controller_max_speed",Float64, callback_speed)

rospy.spin()

GPIO.cleanup()
