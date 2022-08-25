### BEGIN INFO
# Provides:		         Info on can Setup
# Short-Description:	 Use this to guide for can setup on the system.
# Description:  	     This file should be used for info on can
# Version:		         0.1.2
# Assumption:
# Dependency:
### END INFO

Package description
--------------------

This library provides for general control and velocity control of Mahindra E2O over gateway CAN provided by Mahindra. The control here is velocity and other peripheral control of the system.

The library provides ROS Wrapper for CAN, written in C++ over SocketCAN API using can-native api in linux. It is used with sys-tec canusb module USBCANModul1 devices to build packets for communication with car. The library is in c++ and needs ROS and only runs on linux. Please note, you will need to install drivers for CAN. For installing CAN drivers, refer to "installation.txt" under the readme folder.

Make sure that 'vcan0/can0' is up and running before you run the library, you can check this by
    $ ip link
where 'vcan0' is identifier for virtual can device,
and 'can0' is identifier for actual can device connected to PC.

High level functionality provided:
  - Library does logging and storing data for commands received and sent over gateway can. More on this in 'Usage.txt' under readme folder.
  - Library does control of the velocity over the gateway can. More on this in 'Usage.txt' under readme folder.
  - There is also a provision to listen on to vehicle can by adding another usbCAN module. Vehicle CAN gives direct access to ECU of car, what we get through gateway is indirect access to ECU through gateway.

Release Notes
--------------
v0.0.2 :
  * Base for velocity control of car through feedback from gateway.
  * Linking for control of car through ROS topics
  * Keyboard and joystick control of car

v0.0.1 :
  * Base for can driver library with send and recieve modality
  * output to various topics for coming from gateway like speed , gear etc.
  * Packet formation and parsing of received messages
  * Logs in the log folder separately for Rx and Tx.

Requirements
-------------

- ROS (https://www.ros.org)
- Pangolin (https://github.com/stevenlovegrove/Pangolin)
- Wheel Encoder Interface package (available on git server as wheel_encoder_interface.git)
- Utils package (available on git server as utils.git)
- ROS Serial (sudo apt-get install ros-kinetic-serial)
- USB-CANmodul series generation 3 and 4 driver installed (present in setup folder)
- Linux Kernel version >= 2.6.32
- Following kernel options have to be set:
    CONFIG_CAN=m
    CONFIG_CAN_RAW=m
    CONFIG_CAN_BCM=m
    CONFIG_CAN_DEV=m
    CONFIG_CAN_CALC_BITTIMING=y

Those configuration options can be found in /boot/config-*
 you can get the value of * by
	  $ uname -r
 this gives you current linux kernel version used.

- [Optional] Doxygen (Optional)(www.doxygen.org/)
- [Optional] CAN utilities from the SocketCAN repository
    $ git clone git://github.com/linux-can/can-utils/
    $ cd can-utils
    $ make
