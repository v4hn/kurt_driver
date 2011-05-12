#!/bin/sh

if [ "${ACTION}" = "add" ]; then
  sleep 5
  stty -F /dev/ttyUSB0 1000000 line 0 cs8 -onlcr -echo raw min 100 time 2
  echo -e -n "C\rF\rS8\rO\r" > /dev/ttyUSB0
  /home/robot/socketcan/can-utils/slcand $( basename ${DEVNAME} ) can0
  ifconfig can0 up
  exit $?
elif [ "${ACTION}" = "remove" ]; then
  kill -TERM $(cat /var/run/slcand-$( basename ${DEVNAME} ).pid )
  sleep 5
  kill -KILL $(cat /var/run/slcand-$( basename ${DEVNAME} ).pid )
  exit $?
fi
