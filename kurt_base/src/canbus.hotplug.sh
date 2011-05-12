#!/bin/sh

if [ "${ACTION}" = "add" ]; then
  sleep 5
  /home/robot/socketcan/can-utils/slcand $( basename ${DEVNAME} )
  exit $?
elif [ "${ACTION}" = "remove" ]; then
  kill -TERM $(cat /var/run/slcand-$( basename ${DEVNAME} ).pid )
  sleep 5
  kill -KILL $(cat /var/run/slcand-$( basename ${DEVNAME} ).pid )
  exit $?
fi
