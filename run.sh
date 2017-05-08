#!/bin/bash

my_pid=$$

echo "My process ID is $my_pid"

echo "Launching roscore..."
roscore &

pid=$!

sleep 3s
echo "Launching Gazebo..."
sleep 3s
roslaunch pioneer_gazebo world.launch &
pid="$pid $!"

sleep 5s
echo "Launching initialisation parameters..."
sleep 3s
roslaunch pioneer_description robot_description.launch &
pid="$pid $!"
roslaunch stage_assets product_description.launch &
pid="$pid $!"
roslaunch stage_assets generic_sdf_launcher.launch file:=5by5_out.sdf name:=5by5
pid="$pid $!"
roslaunch stage_assets product.launch name:=product1 pose:="-x $(rosparam get /product1/x) -y $(rosparam get /product1/y) -Y $(rosparam get /product1/a)" &
pid="$pid $!"

echo "Launching MapInformation 1..."
rosrun pioneer_control map_information &
pid="$pid $!"

for i in `seq 1 1`;
do
#	echo "Spawning Pioneer $i..."
	roslaunch pioneer_description generic_pioneer.launch name:=pioneer$i pose:="-x $(rosparam get /pioneer$i/x) -y $(rosparam get /pioneer$i/y) -Y $(rosparam get /pioneer$i/a)" &
	#roslaunch pioneer_control robotic_agent_full.launch id:="0" gridpos:="1 0 0 1" &
	##rosrun pioneer_ros pioneer_tf_broadcaster model_name:=pioneer1
	##rosrun pioneer_ros pioneer_odom_publisher model_name:=pioneer1
	pid="$pid $!"
	sleep 5s
done

#sleep 5s
#for i in `seq 1 5`;
#do
#	roslaunch pioneer_control robotic_agent_full.launch id:="$i" gridpos:="$(rosparam get /pioneer$i/px) $(rosparam get /pioneer$i/py) $(rosparam get /pioneer$i/dy) $(rosparam get /pioneer$i/dy)" &
#done
#sleep 5s
#echo "Launching Pioneer 2..."
#roslaunch pioneer_description generic_pioneer.launch name:=pioneer2 pose:="-x 2 -y 2 -Y 1.57" &
#rosrun pioneer_ros pioneer_tf_broadcaster model_name:=pioneer2
#rosrun pioneer_ros pioneer_odom_publisher model_name:=pioneer2
#pid="$pid $!"
trap "echo Killing all processes.; kill -2 TERM $pid; exit" SIGINT SIGTERM

sleep 24h
#for i in `seq 1 5`;
#do
#	roslaunch pioneer_description generic_pioneer.launch name:=pioneer$i pose:="-x $(rosparam get /pioneer$i/x) -y $(rosparam get /pioneer$i/y) -Y $(rosparam get /pioneer$i/a)" &
#	pid="$pid $!"
#	sleep 7s
#done
