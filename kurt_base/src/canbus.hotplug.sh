#!/bin/sh

if [ "${ACTION}" = "add" ]; then
        sleep 5
        slcand $( basename ${DEVNAME} )
        exit $?
elif [ "${ACTION}" = "remove" ]; then
        kill -TERM $(< /var/run/slcand-$( basename ${DEVNAME} ).pid )
        sleep 5
        kill -KILL $(< /var/run/slcand-$( basename ${DEVNAME} ).pid )
        exit $?
fi
