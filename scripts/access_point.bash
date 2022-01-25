#!/usr/bin/env bash
#
# Cory Duce, Nov 26 2020
# Bill Mania updated, 24 Jan 2022
#
# This script at boot to ensure there is an Access Point
# setup for allowing headless and wireless access to the robot.
#

NM_CONN_NAME="roboAP"
AP_PSK="roboquest"
AP_SSID_BASE="roboAP_"

#
# In order to create a unique Access Point number, append
# the last 4 digits of the eth0 MAC address.
#
UNIQUE_MAC=$(ip address show dev eth0 scope link | awk '/ether/{print $2}' | sed 's/://g' | grep -o '....$')
AP_SSID=$AP_SSID_BASE$UNIQUE_MAC

CONNECTION=$(nmcli -c no con show | grep $NM_CONN_NAME)
if [[ $CONNECTION != *"$NM_CONN_NAME"* ]]
then
    logger -p user.notice "Defining Access Point:$AP_SSID"
    nmcli -c no \
	con add \
	type wifi \
	ifname wlan0 \
	con-name $NM_CONN_NAME \
	autoconnect no \
	ssid $AP_SSID
    nmcli -c no \
	con modify $NM_CONN_NAME \
	802-11-wireless.mode ap \
	802-11-wireless.band bg \
	ipv4.method shared
    nmcli -c no \
	con modify $NM_CONN_NAME \
	wifi-sec.key-mgmt wpa-psk
    nmcli -c no \
	con modify $NM_CONN_NAME \
	wifi-sec.psk $AP_PSK
    nmcli -c no \
	con up ifname wlan0
else
        logger -p user.notice  "Access Point:$AP_SSID exists"
fi
