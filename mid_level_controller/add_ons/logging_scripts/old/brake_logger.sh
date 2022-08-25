#!/bin/bash
x=1000
y=500
z=$(calc $x / $y)
echo $z
mode=cycloidal
loop_var=0
 while [ $loop_var -lt 10 ]; do
	y=$(calc $y + 50)
	z=$(calc $x / $y)
	if ! [[ $(echo $z | grep "~") = "" ]]; then
		z=$(echo $z | cut -d '~' -f 2)
		z=$(echo ${z:0:5})
	fi
	echo $z
	rosrun mid_level_controller speed_mapper $z $mode
	let loop_var=loop_var+1
done
