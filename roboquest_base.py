#! /usr/bin/python
#
# roboquest_base.py
# Cory Duce cory.duce@gmail.com
# Nov 24 2020
# 
# v1.0 -Initial release
#
import rospy
import RPi.GPIO as GPIO
from std_msgs.msg import Float32, Bool, Int32
import serial
import os
import subprocess
import NetworkManager
import time

ser = serial.Serial('/dev/ttyAMA0',57600,timeout=0.1)

#Constants
PIN_DRIVER1 = 24
PIN_DRIVER2 = 25
PIN_CHARGER_DETECT = 7
PIN_CHARGER_ENABLE = 21
SHUT_DOWN_TIMER_RELOAD = 10
c = NetworkManager.const

#Globals
screen_pos = 0
oldx1 = 0

#==============================================================
# ROS Subscriber Callback functions

#Enable or disable the charger
def cb_chargerEnable(msg):
	if(msg.data == True):
		GPIO.output(PIN_CHARGER_ENABLE, GPIO.LOW)	#Turn the charger on
	if(msg.data == False):
		GPIO.output(PIN_CHARGER_ENABLE, GPIO.HIGH)	#Turn the charger off

#Enable or disable the FET current driver #1
def cb_driver1(msg):
	if(msg.data == True):
		GPIO.output(PIN_DRIVER1, GPIO.HIGH)	
	if(msg.data == False):
		GPIO.output(PIN_DRIVER1, GPIO.LOW)

#Enable or disable the FET current driver #2
def cb_driver2(msg):
	if(msg.data == True):
		GPIO.output(PIN_DRIVER2, GPIO.HIGH)
	if(msg.data == False):
		GPIO.output(PIN_DRIVER2, GPIO.LOW)

		
#def cb_sleepTime(msg):
#	print("Sleep time is")
#	print(msg.data)
#
#def cb_sleepEnable(msg):
#	print("Sleep Enable")
#	
#
##callbacks for buttons that start and stop a shutdown or sleep sequence
#def cb_shutdownStart(msg):
#	global enterShutdownActive
#	global enterSleepActive
#	
#	if(msg.data == True and enterShutdownActive == False):		#only start the shutdown proceedure if we are currently not preparing to shutdown
#		enterSleepActive = False								#stop a sleep proceedure. Now we're getting ready to shut down
#		enterShutdownActive = True
#		print("Shutdown Sequence Started")
#
#def cb_sleepStart(msg):
#	global enterShutdownActive
#	global enterSleepActive
#	
#	if(msg.data == True and enterSleepActive == False):			#only start the sleep proceedure if we are currently not preparing to sleep
#		enterShutdownActive = False								#stop a shutdown proceedure. Now we're getting getting ready to sleep
#		enterSleepActive = True
#		print("Sleep Sequence Started")
#		
#def cb_shutdownSleepStop(msg):
#	global enterShutdownActive
#	global enterSleepActive
#	
#	if(msg.data == True):
#		enterShutdownActive = False
#		enterSleepActive = False
#		enterShutdownCounter = SHUT_DOWN_SLEEP_COUNTER_RELOAD
#		enterSleepCounter = SHUT_DOWN_SLEEP_COUNTER_RELOAD
#		print("Shutdown and sleep sequence Stopped")
		

		
#Event that runs 10/sec checking for new serial port messages from the Pi
def getSerialPortData(event=None):
	global screen_pos
	global oldx1
	#==============================================================
	#variables for ROS data to be published
	
	batteryVoltage = Float32()				#battery voltage in volts. 12 volt nominal
	batteryCurrent = Float32()				#battery current in mA. Positive when current flowing into battery ie charging, negative when battery is discharging
	systemCurrent = Float32()				#system current in mA. This is current that is powering the robot. Positive when system is running. Should not be negative
	
	ADC0 = Float32()						#ADC inputs. 0-3.3 volts
	ADC1 = Float32()
	ADC2 = Float32()
	ADC3 = Float32()
	ADC4 = Float32()
	
	chargerPwrState = Bool()				#state of the external charger power
	
	#==============================================================
	# The micro controller is sending a bunch of telemetry over the serial port
	# The data is in string format "$$$ value1 value2 ... valueN \n"
	#try:
	msg = ser.readline()
	x = msg.split()
	#print(x)
	if( len(x) > 0):

		if( x[0] == "$$SCREEN" ):	#Message requesting a screen of text

			#remove the $$SCREEN from the start of list, and convert all to float
			x.pop(0)
			print("Show screen {0} {1}".format(x[0],x[1]))
			
			if(x[0] != oldx1):
				oldx1 = x[0]
				screen_pos = 0
				
			if(x[0] == '2'):
				if(x[1] == '0'):
					ser.write("$$screen=2="+get_network_device()+"\r")
				elif(x[1] == '1'):
					screen_pos += 1
					ser.write("$$screen=2="+get_network_device()+"\r")
				elif(x[1] == '-1'):
					screen_pos -= 1
					ser.write("$$screen=2="+get_network_device()+"\r")

			if(x[0] == '3'):				
				if(x[1] == '0'):
					ser.write("$$screen=3="+get_network_connections()+"\r")
				elif(x[1] == '1'):
					screen_pos += 1
					ser.write("$$screen=3="+get_network_connections()+"\r")
				elif(x[1] == '-1'):
					screen_pos -= 1
					ser.write("$$screen=3="+get_network_connections()+"\r")
				elif(x[1] == '2'):
					ser.write("$$screen=3="+padResult(println('Connecting...'))+"\r")
					join_network()
					time.sleep(0.1)
					ser.write("$$screen=3="+get_network_connections()+"\r")
		
		elif( x[0] == "$$TELEM" ):	#Message with telemetry data

			#remove the $$TELEM from the start of list, and convert all to float
			x.pop(0)	
			x = [float(i) for i in x]

			#print(x)
			batteryVoltage.data = x[0]
			batteryCurrent.data = x[1]
			systemCurrent.data = x[2]
			ADC0.data = x[3]
			ADC1.data = x[4]
			ADC2.data = x[5]
			ADC3.data = x[6]
			ADC4.data = x[7]
			chargerPwrState.data = x[8]
			
			
			#publish the data
			pub_batteryVoltage.publish(batteryVoltage)
			pub_batteryCurrent.publish(batteryCurrent)
			pub_systemCurrent.publish(systemCurrent)
			pub_ADC0.publish(ADC0)
			pub_ADC1.publish(ADC1)
			pub_ADC2.publish(ADC2)
			pub_ADC3.publish(ADC3)
			pub_ADC4.publish(ADC4)
			pub_chargerPwrState.publish(chargerPwrState)
		else:
			ser.flushInput()
		
	

	
#def shutdownLoop(event=None):
#	global shutdownTimer
#	global shutdownActive
#	print("shutdown loop")
#	
#	if(enterShutdownActive == True):
#		print("shutdown loop active")
#		shutdownTimer = shutdownTimer -1
#		if(shutdownTimer == 0):
#			#shutdown
#			#os.system('sudo shutdown now')
#			print("Shuting Down")
#	
#	shutdownTimerROS = Int32()				#starts counting down when the shutdown command is received
#	shutdownTimerROS.data = shutdownTimer
#	pub_shutdownCounter.publish(shutdownTimerROS)
	
					
#parses all network device info for device at screen_pos index
def get_network_device():
	global screen_pos
	strength = ''
	try:
		ac = NetworkManager.NetworkManager.ActiveConnections
		conn = ac[screen_pos % len(ac)]
		settings = conn.Connection.GetSettings()
	
		for s in list(settings.keys()):
			if 'data' in settings[s]:
				settings[s + '-data'] = settings[s].pop('data')
	
		secrets = conn.Connection.GetSecrets()
		for key in secrets:
			settings[key].update(secrets[key])
			
		if hasattr(conn,'Devices') and conn.Devices:
			output = println('Name: '+str(conn.Devices[0].Interface))
			output += println('IP: '+str(conn.Devices[0].Ip4Config.Addresses[0][0]))
		
		size = max([max([len(y) for y in list(x.keys()) + ['']]) for x in settings.values()])
		format = "      %%-%ds %%s" % (size + 5)
		for key, val in sorted(settings.items()):
			for name, value in val.items():
				if name == 'ssid':
					output += println('SSID: '+str(value))
					for ap in NetworkManager.AccessPoint.all():
						try:
							if(str(ap.Ssid) == str(value)):
								strength = ap.Strength
						except Exception as e:
							print e
							pass
				elif name == 'psk':
					output += println('Pwd: '+str(value))
				elif name == 'type':
					output += println('Type: '+str(value))
		if(str(strength) != ''):
			strengthString = '  strength:'+str(strength)+'%'
		else:
			strengthString = '';
		output = println(str((screen_pos % len(ac))+1)+'/'+str(len(ac)) + strengthString)+output
		return padResult(output)
	except AttributeError:
		return padResult(println('still connecting...')+println('')+println('press button to')+println('refresh'))
	except:
		return padResult(println('Device error'))

#Parse the list of all available network connections
def get_network_connections():
	global screen_pos
	global gconn
	strength = ''
	cn = NetworkManager.Settings.ListConnections()
	gconn = cn[screen_pos % len(cn)]
	settings = gconn.GetSettings()
	secrets = gconn.GetSecrets()
	for key in secrets:
		settings[key].update(secrets[key])
	output = println('Name: '+str(settings['connection']['id']))
	for key, val in sorted(settings.items()):
		for name, value in val.items():
			if name == 'ssid':
				output += println('SSID: '+str(value))
				for ap in NetworkManager.AccessPoint.all():
					try:
						if(str(ap.Ssid) == str(value)):
							strength = ap.Strength
					except Exception as e:
						print(e)
						pass
			elif name == 'psk':
				output += println('Pwd: '+str(value))
			elif name == 'type':
				output += println('Type: '+str(value))
	if(str(strength) != ''):
		strengthString = '  strength:'+str(strength)+'%'
	else:
		strengthString = '';
	output = println(str((screen_pos % len(cn))+1)+'/'+str(len(cn)) + strengthString)+output
	return padResult(output)

#pads text to clear the screen
def println(text):
	return text.ljust(23, ' ')+'\n'
#pad final result
def padResult(text):
	return (text+(' '*23 + '\n')*4)[0:141];

#Attempts to activate a connection. This will join a wireless network for example
def join_network():
	global gconn
	print('nmcli connection up '+str(gconn.GetSettings()['connection']['id']))
	#os.system('nmcli connection up '+str(gconn.GetSettings()['connection']['id']).replace(' ','\ '))
	subprocess.Popen('nmcli connection up '+str(gconn.GetSettings()['connection']['id']).replace(' ','\ '),shell=True);

	
#==============================================================================================

rospy.init_node('roboquest_base')

GPIO.setmode(GPIO.BCM)

#==============================================================
# Global non ROS variables
#shutdownTimer = SHUT_DOWN_TIMER_RELOAD	#Counts down when shutdownActive is true
#enterShutdownActive = False					#set true to start the shutdown proceedure
#enterSleepActive = False					#set true to start the sleep proceedure

#==============================================================
# Setup the publishers

#System voltage and current
pub_batteryVoltage = rospy.Publisher('voltage_battery',Float32,queue_size = 1)
pub_batteryCurrent = rospy.Publisher('current_battery',Float32,queue_size = 1)
pub_systemCurrent  = rospy.Publisher('current_system',Float32,queue_size = 1)

#General Purpose ADC inputs
pub_ADC0 = rospy.Publisher('voltage_adc_0',Float32,queue_size = 1)
pub_ADC1 = rospy.Publisher('voltage_adc_1',Float32,queue_size = 1)
pub_ADC2 = rospy.Publisher('voltage_adc_2',Float32,queue_size = 1)
pub_ADC3 = rospy.Publisher('voltage_adc_3',Float32,queue_size = 1)
pub_ADC4 = rospy.Publisher('voltage_adc_4',Float32,queue_size = 1)

#Status of external charge power
pub_chargerPwrState = rospy.Publisher('charger_power_state',Bool,queue_size = 1)

#Shutdown counter
#pub_shutdownCounter = rospy.Publisher('shutdownCounter',Int32,queue_size = 1)


#==============================================================
# Setup the Subscribers
rospy.Subscriber("charger_enable",Bool, cb_chargerEnable)	#true/false to enable/disable the charging circuit
#rospy.Subscriber("sleepTime",Int32, cb_sleepTime)			#the number of seconds to sleep the pi after getting a sleepEnable request. This is forwarded on to the micro
#rospy.Subscriber("sleepEnable",Bool, cb_sleepStart)		#when true received signal the micro to put the pi to sleep
#rospy.Subscriber("shutdown",Bool, cb_shutdownStart)				#when true received, start the shutdown counter, when false received, reset and stop the shutdown counter

#The FET drivers
rospy.Subscriber("current_driver_1",Bool, cb_driver1)				#true/false turns FET driver 1 on/off
rospy.Subscriber("current_driver_2",Bool, cb_driver2)				#true/false turns FET driver 2 on/off


#==============================================================
#init all the GPIO

#use the BCM pin numbers for the IO
GPIO.setmode(GPIO.BCM)

GPIO.setup(PIN_DRIVER1, GPIO.OUT)
GPIO.output(PIN_DRIVER1, GPIO.LOW)

GPIO.setup(PIN_DRIVER2, GPIO.OUT)
GPIO.output(PIN_DRIVER2, GPIO.LOW)

GPIO.setup(PIN_CHARGER_ENABLE, GPIO.OUT)
GPIO.output(PIN_CHARGER_ENABLE, GPIO.LOW)		#Low will enable the charger

GPIO.setup(PIN_CHARGER_DETECT, GPIO.IN)

#==============================================================
#Everthing is setup and ready to run handlers and event loops

ser.flushInput()

rospy.Timer(rospy.Duration(1.0/10.0), getSerialPortData)			#get data from the serial port 10/sec
#rospy.Timer(rospy.Duration(1.0/1.0), shutdownLoop)

rospy.spin()

GPIO.cleanup()
