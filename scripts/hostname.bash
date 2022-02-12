#!/usr/bin/env bash
#
# Bill Mania, 12 Feb 2022
#
# Executed at boot to ensure the hostname is unique,
# before requesting an IP address from the DHCP service.
#

HOSTNAME_BASE="rq"

#
# In order to create a unique hostname, append
# the last 5 digits of the machine ID.
#
UNIQUE_HOST=$(grep -o '.....$' /etc/machine-id)
HOSTNAME="$HOSTNAME_BASE-$UNIQUE_HOST"

hostnamectl set-hostname "$HOSTNAME"
hostnamectl set-chassis embedded
hostnamectl set-deployment development
hostnamectl set-location mobile

logger -p user.notice  "Hostname set to $HOSTNAME"

exit 0
