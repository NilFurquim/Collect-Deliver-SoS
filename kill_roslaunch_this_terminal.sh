ps -o pid,comm | grep roslaunch | grep -oP '[0-9]+' | while read -r line ; do
	kill $line
done

