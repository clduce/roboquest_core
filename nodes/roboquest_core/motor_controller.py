#!/usr/bin/env python3
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
import Jetson.GPIO as GPIO
import time
from textwrap import wrap

PIN_MOTOR_ENABLE = 17

#
# Raspberry Pi 4B
#MOTOR_I2C_BUS = 6
# Jetson Nano dev kit J41
MOTOR_I2C_BUS = 0

bus = smbus.SMBus(MOTOR_I2C_BUS)
DEVICE_ADDRESS = 0x53
speed = 100.0
enabledState = False


#==============================================================
# ROS Subscriber Callback functions

def callback_speed(msg):
	global speed
	speed = msg.data
	
def callback_enable(msg):
	global enabledState
	if(msg.data == True):
		GPIO.output(PIN_MOTOR_ENABLE, GPIO.HIGH)
		time.sleep(0.2)
		enabledState = True

	if(msg.data ==False):
		GPIO.output(PIN_MOTOR_ENABLE, GPIO.LOW)
		enabledState = False

	pub_motorPwrState.publish(enabledState)

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

	
	#only send I2C commands to the motor controller if is enabled, ie powered up
	if(enabledState):
		#attempt to write to the I2C registers. Try max 3 times. Power off the motor
		#controller if all three fail. 
		success = False
		loops = 3
		while(not success and loops > 0 ):
			try:
				#Registers 3 and 4 hold the individual motor speed commands
				bus.write_i2c_block_data(DEVICE_ADDRESS,3,int_to_byte_array(right))
				bus.write_i2c_block_data(DEVICE_ADDRESS,4,int_to_byte_array(left))
				success = True
			except:
				loops -= 1
				print("Motor I2C error. Loop {0}".format(loops))
				time.sleep(.01)

		if(loops == 0):
			print("Unable to communicate with Motors. Turning off Motor Power")
			GPIO.output(PIN_MOTOR_ENABLE, GPIO.LOW)	#disable the motor driver
			enabledState = False
			pub_motorPwrState.publish(False)


	



#===================================================================================
#init the node with a name
rospy.init_node('motor_controller', anonymous=True)


#==============================================================
# Setup the publishers
pub_motorPwrState = rospy.Publisher('motor_controller_power_enable_state',Bool,queue_size = 1, latch = True)


#==============================================================
# Setup the Subscribers
rospy.Subscriber("drive_joystick",Vector3, callback_drivejs)
rospy.Subscriber("motor_controller_power_enable",Bool, callback_enable)
rospy.Subscriber("motor_controller_max_speed",Float64, callback_speed)

#==============================================================
#init all the GPIO
GPIO.setmode(GPIO.BCM)
GPIO.setup(PIN_MOTOR_ENABLE, GPIO.OUT)

GPIO.output(PIN_MOTOR_ENABLE, GPIO.LOW)
pub_motorPwrState.publish(False)

#==============================================================
#Everthing is setup and ready to run handlers
rospy.spin()

GPIO.cleanup()
