#!/bin/bash
rosbag record -b 0 \
	/gps/fix \
	/imu/data \
	/imu/mag \
	/odometry/wheel_encoder
	/ublox_gps/fix \
	/velocitySteeringAngle \
  /wheelEnc/right \
  /wheekEnc/left \
  /canControl \
  /canStatus
