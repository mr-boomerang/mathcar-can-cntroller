#!/bin/bash
# color printing
txtred=$(tput setaf 1) # Red
txtgrn=$(tput setaf 2) # Green
txtylw=$(tput setaf 3) # Yellow
txtrst=$(tput setaf sgr0) # Reset
# get package location
location=$(rospack find mid_level_controller)
echo $location
if [ "$location" = "" ] ; then
  printf "$txtred Could not find mid level controller.. exiting.\n $txtrst"
  exit 0
fi

# print message
echo "$txtylw Ensure that arguments are valid. They are passed as is. $txtrst"
# create logs folder
mkdir -p "$location/logs"
# setup arguments
inputs_file_name="control_messages.yaml"
status_file_name="status_messages.yaml"
log_file_path="./logs/"
settings_file_name="settings_default.cfg"
# print message
echo "$txtgrn Starting mid level controller...$txtrst"
# run the MLC
rosrun mid_level_controller mid_level_controller --control_yaml_file_path $inputs_file_name --log_path $log_file_path --cfg $settings_file_name --status_yaml_file_path $status_file_name $@
#sleep 5
#rosrun mid_level_controller dataplotter

#rosrun mid_level_controller mid_level_controller --control_yaml_file_path control_messages.yaml --log_path ./logs --cfg settings_default.cfg --status_yaml_file_path status_messages.yaml
