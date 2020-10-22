#!/bin/bash

number_of_nodes=$(rosparam get /num_of_robots)
echo "Launching $number_of_nodes twist_mux nodes..."

trap 'killall' INT

killall() {
    trap '' INT TERM     # ignore INT and TERM while shutting down
    echo "**** Shutting down... ****"     # added double quotes
    kill -TERM 0         # fixed order, send TERM not INT
    wait
    echo DONE
}

for ((i=0; i<$number_of_nodes; i++));
do
	roslaunch sphero_bringup twist_mux.launch namespace:="sphero_$i"
done
echo "DONE"

cat
