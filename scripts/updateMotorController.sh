#!/usr/bin/env bash
echo "17" > /sys/class/gpio/export
echo "out" > /sys/class/gpio/gpio17/direction
echo "1" > /sys/class/gpio/gpio17/value
sleep 0.2
#./twiboot -a 0x29 -d /dev/i2c-6 -w flash:RoboQuestMotorControllerV5-2.hex
./twiboot -a 0x29 -d /dev/i2c-6 -w flash:$1
sleep 1
echo "0" > /sys/class/gpio/gpio17/value

