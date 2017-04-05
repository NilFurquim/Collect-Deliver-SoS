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

sleep 7s
echo "Launching initialisation parameters..."
sleep 3s
roslaunch pioneer_description robot_description.launch &
roslaunch stage_assets product_description.launch &
roslaunch stage_assets generic_sdf_launcher.launch file:=5by5_out.sdf name:=5by5

sleep 5s
echo "Launching MapInformation 1..."
rosrun pioneer_control map_information &
pid="$pid $!"


sleep 5s
echo "Launching Pioneer 1..."
roslaunch pioneer_description generic_pioneer.launch name:=pioneer1 pose:="-x $(rosparam get /pioneer1/x) -y $(rosparam get /pioneer1/y) -Y $(rosparam get /pioneer1/a)" &
#rosrun pioneer_ros pioneer_tf_broadcaster model_name:=pioneer1
#rosrun pioneer_ros pioneer_odom_publisher model_name:=pioneer1
pid="$pid $!"

#sleep 5s
#echo "Launching Pioneer 2..."
#roslaunch pioneer_description generic_pioneer.launch name:=pioneer2 pose:="-x 2 -y 2 -Y 1.57" &
#rosrun pioneer_ros pioneer_tf_broadcaster model_name:=pioneer2
#rosrun pioneer_ros pioneer_odom_publisher model_name:=pioneer2
#pid="$pid $!"

sleep 2s
echo "Launching product spawner..."
roslaunch stage_assets product.launch name:=product1 pose:="-x $(rosparam get /product1/x) -y $(rosparam get /product1/y) -Y $(rosparam get /product1/a)" &
pid="$pid $!"

trap "echo Killing all processes.; kill -2 TERM $pid; exit" SIGINT SIGTERM

sleep 24h
#for i in `seq 1 5`;
#do
#	roslaunch pioneer_description generic_pioneer.launch name:=pioneer$i pose:="-x $(rosparam get /pioneer$i/x) -y $(rosparam get /pioneer$i/y) -Y $(rosparam get /pioneer$i/a)" &
#	pid="$pid $!"
#	sleep 7s
#done
