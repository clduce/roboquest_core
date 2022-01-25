#!/usr/bin/env python3

import RPi.GPIO as GPIO
from os import system
from sys import exit

PIN_MCU_HANDSHAKE_1 = 27
GPIO.setmode(GPIO.BCM)
GPIO.setup(PIN_MCU_HANDSHAKE_1, GPIO.IN)

while True:
    if GPIO.input(PIN_MCU_HANDSHAKE_1):
        system("shutdown -h now")
        exit(0)
