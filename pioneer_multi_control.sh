#!/bin/bash

pid=""
for i in `seq 1 3`;
do
	roslaunch pioneer_control robotic_agent_full.launch id:="$i" gridpos:="$(rosparam get /pioneer$i/px) $(rosparam get /pioneer$i/py) $(rosparam get /pioneer$i/dx) $(rosparam get /pioneer$i/dy)" &
	pid="$pid $!"
	sleep 2s
done

trap "echo Killing all processes.; kill -2 TERM $pid; exit" SIGINT SIGTERM
