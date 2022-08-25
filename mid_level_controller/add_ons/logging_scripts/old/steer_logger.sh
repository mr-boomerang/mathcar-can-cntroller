#!/bin/bash
x=1000
y=2000
z=$(calc $x / $y)
loop_var=0
 while [ $loop_var -lt 10 ]; do 
	y=$(calc $y + 100)
	z=$(calc $x / $y) 
	if ! [[ $(echo $z | grep "~") = "" ]]; then
		z=$(echo $z | cut -d'~' -f 2)
		z=${z:0:5}
	fi
	echo $z
	let loop_var=loop_var+1
#	rosrun can_driver steer_mapper $z
done
