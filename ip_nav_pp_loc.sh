echo "Launching image_processing..."
roslaunch pioneer_control generic_ns_launcher.launch ns:=$1 name:=image_processing &
pid=$!

sleep 2s
echo "Launching localizaton.."
roslaunch pioneer_control generic_ns_launcher.launch ns:=$1 name:=localization &
pid="$pid $!"

sleep 2s
echo "Launching path_planning..."
roslaunch pioneer_control generic_ns_launcher.launch ns:=$1 name:=path_planning &
pid="$pid $!"

sleep 2s
echo "Launching navigation..."
roslaunch pioneer_control generic_ns_launcher.launch ns:=$1 name:=navigation
pid="$pid $!"


trap "echo Killing all processes.; kill -2 TERM $pid; exit" SIGINT SIGTERM

sleep 24h
