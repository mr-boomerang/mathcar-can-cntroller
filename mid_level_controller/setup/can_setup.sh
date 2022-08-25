#!/bin/bash
if [ "$1" == "-h" ] || [ "$1" == "" ] ; then
  echo -e "Usage: 'sudo ./can_setup.sh [OPTIONS]' "
  echo -e "where OPTIONS := "
  echo -e "\t-h\tShow help"
  echo -e "\t-v\tSetup Virtual can at vcan0"
  echo -e "\t-i\tInstall dependencies"
  echo -e "\t-c\tSetup can at can0 (Only if hardware is present and connected)"
  exit 0
fi

if [ "$1" == "-v" ]; then
	cp ./can_virtual.conf ~/.canrc
	sudo modprobe vcan
	sudo ip link add vcan0 type vcan
	sudo ip link set vcan0 up
fi

if [ "$1" == "-c" ]; then
	cp ./can_real.conf ~/.canrc
	sudo modprobe can
	sudo modprobe can_raw
	sudo modprobe can_dev
	sudo ip link add can0 type can
	sudo ip link set can0 up typr can bitrate 500000
fi

if [ "$1" == "-i" ]; then
	tar xvfj systec_can-V0.9.4.tar.bz2
	cd systec_can
	make
	sudo make firmware_install
	sudo make modules_install
	sudo insmod systec_can.ko
	sudo ip link set can0 up
	modules_entered=$(cat /etc/modules | grep 'can')
	if [ "$modules_entered" = "" ]; then
		echo "can" | sudo tee /etc/modules
	fi
	modules_entered=$(cat /etc/modules | grep 'can')
	if [ "$modules_entered" = "" ]; then
		echo "can" | sudo tee /etc/modules
		echo "can_raw" | sudo tee -a /etc/modules
		echo "can_dev" | sudo tee -a /etc/modules
	fi
fi
