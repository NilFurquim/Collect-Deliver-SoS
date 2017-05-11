#!/bin/bash

my_pid=$$

if [ $1 = "" ]; then
	echo "arg1 Provide number of robots"
	exit 1
fi

if [ $2 = "" ]; then
	echo "arg1 Provide test file name"
	exit 1
fi

if [ ! -f tests/$2.test ]; then
	echo "File not found!"
	exit 1
fi

echo "My process ID is $my_pid"

echo "Launching roscore..."
roscore &

pid=$!

sleep 3s
echo "Launching Gazebo..."
sleep 3s
roslaunch pioneer_gazebo world.launch gui:=false&
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
for i in `seq 1 5`;
do
	roslaunch stage_assets product.launch name:=product$i pose:="-x $(rosparam get /product$i/x) -y $(rosparam get /product$i/y) -Y $(rosparam get /product$i/a)" &
	pid="$pid $!"
done

echo "Launching MapInformation 1..."
rosrun pioneer_control map_information &
pid="$pid $!"
sleep 1s
current_date_time="`date +%Y.%m.%d.%H.%M.%S`";
echo "$2_$1_$current_date_time.out"
rosrun pioneer_control product_manager 2> tests/$2_$1_$current_date_time.out&
pid="$pid $!"
sleep 1s

for i in `seq 1 $1`;
do
#	echo "Spawning Pioneer $i..."
	roslaunch pioneer_description generic_pioneer.launch name:=pioneer$i pose:="-x $(rosparam get /pioneer$i/x) -y $(rosparam get /pioneer$i/y) -Y $(rosparam get /pioneer$i/a)" &
	#roslaunch pioneer_control robotic_agent_full.launch id:="0" gridpos:="1 0 0 1" &
	##rosrun pioneer_ros pioneer_tf_broadcaster model_name:=pioneer1
	##rosrun pioneer_ros pioneer_odom_publisher model_name:=pioneer1
	pid="$pid $!"
	sleep 1s
done


echo "Launching pioneer control..."
sleep 10s
for i in `seq 1 $1`;
do
	roslaunch pioneer_control robotic_agent_full.launch id:="$i" gridpos:="$(rosparam get /pioneer$i/px) $(rosparam get /pioneer$i/py) $(rosparam get /pioneer$i/dx) $(rosparam get /pioneer$i/dy)" &
	pid="$pid $!"
	sleep 2s
done

sleep 5s
rosrun pioneer_control application < tests/$2.test
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
