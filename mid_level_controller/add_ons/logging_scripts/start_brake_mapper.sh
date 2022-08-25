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
  printf "$txtred Could not find mid level controller.. exiting.\n $txtwht"
  exit 0
fi
# create logs folder
mkdir -p "$location/logs"
# setup arguments
settings_file_path="settings_default.cfg"
# print message
echo "$txtgrn Starting mid level controller...$txtwht"
# run the transceiver
rosrun mid_level_controller mid_level_controller_node --cfg $settings_file_path &
sleep 5
# rosrun mid_level_controller brake_mapper 
