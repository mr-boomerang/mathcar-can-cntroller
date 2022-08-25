#!/bin/bash

if [ "$1" == "" ]
then
  DEV="can0"
else
  DEV="$1"
fi

ip link set $DEV type can bitrate 125000
ifconfig $DEV up

