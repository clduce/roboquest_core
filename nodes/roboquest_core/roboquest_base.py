#!/usr/bin/env python3
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

rospy.init_node('roboquest_base')

hat_port = rospy.get_param('~hat_port')
hat_data_rate = rospy.get_param('~hat_data_rate')
rospy.logdebug(f"roboquest_base parameters: hat_port:{hat_port}, hat_data_rate:{hat_data_rate}")
hat = serial.Serial(hat_port, hat_data_rate, timeout=0.1)
rospy.logdebug("Serial port opened")

PIN_DRIVER1 = 24
PIN_DRIVER2 = 25
PIN_CHARGER_DETECT = 7
PIN_CHARGER_ENABLE = 21
HAT_MCU_CONTROL_PIN = 22        
SHUT_DOWN_TIMER_RELOAD = 10
ENABLE = GPIO.HIGH
DISABLE = GPIO.LOW
c = NetworkManager.const

#
# Screen IDs
#
HOME = '1'
DEVICES = '2'
CONNECTIONS = '3'
MISC = '4'
ABOUT = '5'

#
# Button IDs
#
DOWN = '-1'
UP = '1'
ENTER = '2'
NO_BUTTON = '0'

#Globals
screen_page = 0
oldx1 = 0

#==============================================================
# ROS Subscriber Callback functions

#Enable or disable the charger
def cb_chargerEnable(msg):
    if(msg.data == True):
        GPIO.output(PIN_CHARGER_ENABLE, GPIO.LOW)    #Turn the charger on
    if(msg.data == False):
        GPIO.output(PIN_CHARGER_ENABLE, GPIO.HIGH)    #Turn the charger off

    pub_chargerEnabled.publish(msg.data)



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

#Puts the robot to sleep for given number of minutes        
def cb_sleepTime(msg):
    hat.write(
        bytearray(
            "$$sleepTime={0}".format(msg.data),
            'ascii'))
    hat.write(
        bytearray(
            "$$sleep",
            'ascii'))
        
#Event that runs 10/sec checking for new serial port messages from the Pi
def getSerialPortData(event=None):
    """
    $$SCREEN <screen ID> <button ID>\r

    <screen ID>
    1 Home - controlled by HAT
    2 Network devices - controlled by Pi
    3 Network connections - controlled by Pi
    4 Network et cetera (unused) - controlled by Pi
    5 About - controlled by HAT

    <button ID>
    -1 down
    0 <no button pressed>
    1 up
    2 center (Enter)
    """

    global screen_page
    global oldx1

    #==============================================================
    #variables for ROS data to be published
    
    battery_volts = Float32()
    battery_milliamps = Float32()                #battery current in mA. Positive when current flowing into battery ie charging, negative when battery is discharging
    systemCurrent = Float32()                #system current in mA. This is current that is powering the robot. Positive when system is running. Should not be negative
    
    ADC0 = Float32()                        #ADC inputs. 0-3.3 volts
    ADC1 = Float32()
    ADC2 = Float32()
    ADC3 = Float32()
    ADC4 = Float32()
    
    chargerPwrState = Bool()                #state of the external charger power
    
    #==============================================================
    # The micro controller is sending a bunch of telemetry over the serial port
    # The data is in string format "$$$ value1 value2 ... valueN \n"
    #try:
    msg = hat.readline().decode('ascii')
    fields = msg.split()
    if fields:
        msg_type = fields[0]

        if msg_type == '$$SCREEN':
            screen_id = fields[1]
            button_id = fields[2]

            GPIO.output(HAT_MCU_CONTROL_PIN, DISABLE)

            if screen_id != oldx1:
                oldx1 = screen_id
                screen_page = 0
                
            if(screen_id == DEVICES):
                if button_id == NO_BUTTON:
                    hat.write(
                        bytearray(
                            "$$screen=2="+get_network_device()+"\r",
                            'ascii'))
                elif button_id == UP:
                    screen_page += 1
                    hat.write(
                        bytearray(
                            "$$screen=2="+get_network_device()+"\r",
                            'ascii'))
                elif button_id == DOWN:
                    screen_page -= 1
                    hat.write(
                        bytearray(
                            "$$screen=2="+get_network_device()+"\r",
                            'ascii'))

            if screen_id == CONNECTIONS:                
                if button_id == NO_BUTTON:
                    hat.write(
                        bytearray(
                            "$$screen=3="+get_network_connections()+"\r",
                            'ascii'))
                elif button_id == UP:
                    screen_page += 1
                    hat.write(
                        bytearray(
                            "$$screen=3="+get_network_connections()+"\r",
                            'ascii'))
                elif button_id == DOWN:
                    screen_page -= 1
                    hat.write(
                        bytearray(
                            "$$screen=3="+get_network_connections()+"\r",
                            'ascii'))
                elif button_id == ENTER:
                    hat.write(
                        bytearray(
                            "$$screen=3="+padResult(println('Connecting...'))+"\r",
                            'ascii'))
                    join_network()
                    time.sleep(0.1)
                    hat.write(
                        bytearray(
                            "$$screen=3="+get_network_connections()+"\r",
                            'ascii'))

            GPIO.output(HAT_MCU_CONTROL_PIN, ENABLE)
        
        elif msg_type == '$$TELEM':
            rospy.logdebug_throttle(5, f"Telem")

            #remove the $$TELEM from the start of list, and convert all to float
            fields.pop(0)    

            try:
                telemetry = [float(i) for i in fields]

                battery_volts.data = telemetry[0]
                battery_milliamps.data = telemetry[1]
                systemCurrent.data = telemetry[2]
                ADC0.data = telemetry[3]
                ADC1.data = telemetry[4]
                ADC2.data = telemetry[5]
                ADC3.data = telemetry[6]
                ADC4.data = telemetry[7]
                chargerPwrState.data = True if telemetry[8] else False

            except Exception as e:
                rospy.logwarn(f"Failed to extract all 9: {e}")
            
            try:
                pub_batteryVoltage.publish(battery_volts)
                pub_batteryCurrent.publish(battery_milliamps)
                pub_systemCurrent.publish(systemCurrent)
                pub_ADC0.publish(ADC0)
                pub_ADC1.publish(ADC1)
                pub_ADC2.publish(ADC2)
                pub_ADC3.publish(ADC3)
                pub_ADC4.publish(ADC4)
                pub_chargerPwrState.publish(chargerPwrState)

            except Exception as e:
                rospy.logwarn(f"Failed to publish telemetry: {e}")
            
        else:
            rospy.logdebug_throttle(5, f"Nothing")

        hat.flushInput()
    
def get_network_device():
    """
    Get the NM Connections which are currently active, ie. applied to a
    NM Device (interface). Collect the details about each NM Device and
    return it to the display, so it can be used to connect with a specific
    network interface.
    """

    strength = ''

    try:
        active_connections = NetworkManager.NetworkManager.ActiveConnections
        if not active_connections:
            rospy.logwarn("No active network Connections")
            return padResult(println("No active Connections"))

        connection = active_connections[screen_page % len(active_connections)]
        settings = connection.Connection.GetSettings()
    
        for setting in settings:
            if 'data' in settings[setting]:
                settings[setting + '-data'] = settings[setting].pop('data')
    
        rospy.logdebug("updated settings")

        try:
            #
            # Retrieve the PSK for this NM Connection, so it can
            # be shown on the UI display.
            #
            secrets = connection.Connection.GetSecrets()

        except Exception as e:
            rospy.logwarn(f"Exception from GetSecrets(): {e}")
            return padResult(println("GetSecrets error"))

        for key in secrets:
            settings[key].update(secrets[key])
            
        rospy.logdebug("got secrets")

        if hasattr(connection, 'Devices') and connection.Devices:
            output = println('Name: '+str(connection.Devices[0].Interface))
            output += println('IP: '+str(connection.Devices[0].Ip4Config.Addresses[0][0]))
        
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
                            rospy.logwarn(
                                f"getNetworkDevice NM.AccessPoint exception {e}")
                            pass

                elif name == 'psk':
                    output += println('Pwd: '+str(value))

                elif name == 'type':
                    output += println('Type: '+str(value))

        if(str(strength) != ''):
            strengthString = '  strength:'+str(strength)+'%'
        else:
            strengthString = '';
        output = println(str(
            (screen_page % len(active_connections))+1)
            +'/'+str(len(active_connections))
            + strengthString) + output
        return padResult(output)

    except AttributeError:
        return padResult(println('still connecting...')+println('')+println('press button to')+println('refresh'))

    except Exception as e:
        rospy.logwarn(f"getNetworkDevice exception: {e}")
        return padResult(println('Device error'))

def get_network_connections():
    """
    Parse the list of all defined NM Connections.
    """

    global gconn

    strength = ''
    all_connections = NetworkManager.Settings.ListConnections()
    if not all_connections:
        return padResult("No network connections")

    gconn = all_connections[screen_page % len(all_connections)]
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
                        rospy.logwarn(f"NM.AccessPoint exception: {e}")
                        pass

            elif name == 'psk':
                output += println('Pwd: '+str(value))
            elif name == 'type':
                output += println('Type: '+str(value))
    if(str(strength) != ''):
        strengthString = '  strength:'+str(strength)+'%'
    else:
        strengthString = '';
    output = println(str((screen_page % len(all_connections))+1)+'/'+str(len(all_connections)) + strengthString)+output
    rospy.logdebug("--S--" + output + "--E--")

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
    rospy.logdebug(
        'nmcli connection up '+str(gconn.GetSettings()['connection']['id']))
    #os.system('nmcli connection up '+str(gconn.GetSettings()['connection']['id']).replace(' ','\ '))
    subprocess.Popen('nmcli connection up '+str(gconn.GetSettings()['connection']['id']).replace(' ','\ '),shell=True);

pub_batteryVoltage = rospy.Publisher('voltage_battery', Float32, queue_size = 1)
pub_batteryCurrent = rospy.Publisher('current_battery', Float32, queue_size = 1)
pub_systemCurrent  = rospy.Publisher('current_system', Float32, queue_size = 1)

#General Purpose ADC inputs
pub_ADC0 = rospy.Publisher('voltage_adc_0',Float32,queue_size = 1)
pub_ADC1 = rospy.Publisher('voltage_adc_1',Float32,queue_size = 1)
pub_ADC2 = rospy.Publisher('voltage_adc_2',Float32,queue_size = 1)
pub_ADC3 = rospy.Publisher('voltage_adc_3',Float32,queue_size = 1)
pub_ADC4 = rospy.Publisher('voltage_adc_4',Float32,queue_size = 1)

#Status of external charge power
pub_chargerPwrState = rospy.Publisher('charger_power_state',Bool,queue_size = 1,  latch = True)

#status of the state of the charger
pub_chargerEnabled = rospy.Publisher('charger_enabled_state',Bool,queue_size = 1, latch = True)

#Shutdown counter
#pub_shutdownCounter = rospy.Publisher('shutdownCounter',Int32,queue_size = 1)


#==============================================================
# Setup the Subscribers
rospy.Subscriber("charger_enable",Bool, cb_chargerEnable)    #true/false to enable/disable the charging circuit
#rospy.Subscriber("sleepTime",Int32, cb_sleepTime)            #the number of seconds to sleep the pi after getting a sleepEnable request. This is forwarded on to the micro
#rospy.Subscriber("sleepEnable",Bool, cb_sleepStart)        #when true received signal the micro to put the pi to sleep
#rospy.Subscriber("shutdown",Bool, cb_shutdownStart)                #when true received, start the shutdown counter, when false received, reset and stop the shutdown counter

#The FET drivers
rospy.Subscriber("current_driver_1",Bool, cb_driver1)                #true/false turns FET driver 1 on/off
rospy.Subscriber("current_driver_2",Bool, cb_driver2)                #true/false turns FET driver 2 on/off


#==============================================================
#init all the GPIO

rospy.logdebug("GPIO setup")
#use the BCM pin numbers for the IO
GPIO.setmode(GPIO.BCM)

GPIO.setup(PIN_DRIVER1, GPIO.OUT)
GPIO.output(PIN_DRIVER1, GPIO.LOW)

GPIO.setup(PIN_DRIVER2, GPIO.OUT)
GPIO.output(PIN_DRIVER2, GPIO.LOW)

GPIO.setup(PIN_CHARGER_ENABLE, GPIO.OUT)
GPIO.output(PIN_CHARGER_ENABLE, GPIO.LOW)        #Low will enable the charger

GPIO.setup(PIN_CHARGER_DETECT, GPIO.IN)

GPIO.setup(HAT_MCU_CONTROL_PIN, GPIO.OUT)
GPIO.output(HAT_MCU_CONTROL_PIN, ENABLE)

#==============================================================
#Everthing is setup and ready to run handlers and event loops

hat.flushInput()

rospy.Timer(rospy.Duration(1.0/10.0), getSerialPortData)            #get data from the serial port 10/sec

#The charger IO is set to turn the charger on by default. Publish this as well
pub_chargerEnabled.publish(True)

rospy.spin()

GPIO.cleanup()
