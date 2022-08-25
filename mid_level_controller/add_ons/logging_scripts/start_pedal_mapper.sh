#!/bin/bash
# color printing
txtred=$(tput setaf 1) # Red
txtgrn=$(tput setaf 2) # Green
txtwht=$(tput setaf 7) # white
txtrst=$(tput setaf sgr0) # Reset
# get package location
location=$(rospack find mid_level_controller)
echo $location
if [ "$location" = "" ] ; then
  printf "$txtred Could not find mid level controller package Please source the setup.bash under devel folder... exiting.\n $txtwht"
  exit 0
fi
# create logs folder
mkdir -p "$location/logs"
# setup arguments
#in percent last value of pedal
pedal_init=13
#in percent upto what value you want to go.
pedal_limit=70
# in kmph what is the max velocity you want to go.
vel_limit=40
# in degree -99 says hold last value, else go the the input value, +ve-> left, -ve -> right
steer=20
# in secs to collect data for
time_limit=30
# gear value neutral, fwddrive, reverse, boost to collect data for.
gear=fwddrive

# assitance message
echo "Starting the Logging for pedal value at:" $pedal_init
echo "Limiting Max. Pedal:" $pedal_limit
echo "Limiting Max. Speed:" $vel_limit
echo "Limiting Max. Speed:" $vel_limit
echo "Target Steer:" $steer
echo "Gear value:" $gear

# print message
echo "$txtgrn Starting Pedal Logging...$txtwht"
rosrun mid_level_controller pedal_mapper $pedal_init $pedal_limit $vel_limit $steer $time_limit $gear
# echo "$txtgrn Starting transceiver...$txtrst"
# run the transceiver
# rosrun mid_level_controller transceiver --control_yaml_file_path $inputs_file_name --log_path $log_file_path --cfg $settings_file_name --status_yaml_file_path $status_file_name &
# sleep 5
# rosrun mid_level_controller dataplotter

# rosrun mid_level_controller mid_level_controller --control_yaml_file_path control_messages.yaml --log_path ./logs --cfg settings_default.cfg --status_yaml_file_path status_messages.yaml
