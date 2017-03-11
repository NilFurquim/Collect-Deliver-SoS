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

sleep 5s
echo "Launching Pioneer 1..."
roslaunch pioneer_description generic_pioneer.launch name:=pioneer1 pose:="-x 1 -y 1 -Y 1.57" &
rosrun pioneer_ros pioneer_tf_broadcaster model_name:=pioneer1
rosrun pioneer_ros pioneer_odom_publisher model_name:=pioneer1
pid="$pid $!"

#sleep 5s
#echo "Launching Pioneer 2..."
#roslaunch pioneer_description generic_pioneer.launch name:=pioneer2 pose:="-x 2 -y 0 -Y 1.57" &
#pid="$pid $!"

sleep 2s
echo "Launching product spawner..."
roslaunch stage_assets product.launch&
pid="$pid $!"

trap "echo Killing all processes.; kill -2 TERM $pid; exit" SIGINT SIGTERM

sleep 24h
