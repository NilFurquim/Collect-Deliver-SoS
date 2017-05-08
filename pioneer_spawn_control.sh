echo "Launching Pioneer $1..."
roslaunch pioneer_description generic_pioneer.launch name:=pioneer$1 pose:="-x $(rosparam get /pioneer$1/x) -y $(rosparam get /pioneer$1/y) -Y $(rosparam get /pioneer$1/a)" &
roslaunch pioneer_control pioneer_control_full.launch id:="0" gridpos:="1 0 0 1" &
#rosrun pioneer_ros pioneer_tf_broadcaster model_name:=pioneer1
#rosrun pioneer_ros pioneer_odom_publisher model_name:=pioneer1
