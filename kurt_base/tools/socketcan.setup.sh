#!/bin/bash

# socketcan.setup.sh:
# Sets options for canusb/peak adapter. These commands moved into startup
# script in kbs-socketcan debian package (canusb.hotplug.sh). If kbs-socketcan
# is correctly installed, that script should be run automatically. This script
# here is just for manually setting up the adapter.

DEVNAME=/dev/ttyUSB
IFNAME=can0

stty -F ${DEVNAME} 1000000 line 0 cs8 -onlcr -echo raw min 100 time 2
echo -e -n "C\rF\rS8\rO\r" > ${DEVNAME}
sudo /opt/kbs-socketcan/slcand $( basename ${DEVNAME} ) ${IFNAME}
sudo ifconfig ${IFNAME} up

#peak can usb set speed echo "i 0x0014 e" > /dev/pcan32
