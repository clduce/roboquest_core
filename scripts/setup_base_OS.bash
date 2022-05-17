#!/usr/bin/env bash

mkdir -p /home/ubuntu/catkin_ws/src
cd /home/ubuntu/catkin_ws/src
git clone https://github.com/clduce/roboquest_core
cd roboquest_core

sudo cp scripts/10-network-manager.pkla /etc/polkit-1/localauthority/50-local.d/

sudo nmcli -c no \
    conn add \
    type ethernet \
    ifname eth0 \
    con-name Ethernet autoconnect yes ipv4.method auto

sudo cp scripts/roboquest-hostname.service /etc/systemd/system/

chmod 0664 /etc/systemd/system/roboquest*service
sudo systemctl enable roboquest-hostname.service
sudo systemctl daemon-reload

#
# Waiting for a WiFi interface.
#
#sudo cp scripts/roboquest-network.service /etc/systemd/system/
