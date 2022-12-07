#!/usr/bin/env python3
# -*- coding: utf-8 -*-
from __future__ import print_function, division
import os
import sys
import math
import yaml
import roslaunch
import rospy
import rospkg


def create_launch_file(file_path, bluetooth_names):
    with open(file_path, 'w+') as launch_file:
        launch_file.write('<launch>\n')
        launch_file.write('<env name="ROSCONSOLE_FORMAT" value="[${severity}] [${node}]> ${message}"/>\n\n')
        for i, name in enumerate(bluetooth_names):
            launch_file.write(f'<node pkg="sphero_driver_v2" type="sphero_node.py" name="sphero_driver" ns="sphero_{i}" output="screen">\n')
            launch_file.write(f'<param name="name" value="{name}"/>\n')
            launch_file.write('</node>\n\n')
        launch_file.write('</launch>\n')


def main():
    try:
        num_of_robots = int(sys.argv[1])
    except Exception as e:
        print("\033[31m You must provide the number of robots as argument.\033[0m")
        print(f"Btw., I cought this exception\n{e}")
    
    # Set up launch variables. These are hard-coded and they shouldn't be changed.
    package = rospkg.RosPack().get_path('sphero_driver_v2')
    sphero_list = package + '/cfg/sphero_addresses.txt'
    launch_file = package + '/launch/drivers.launch.xml'
    with open(sphero_list, 'r') as stream:
        lines = [next(stream) for _ in range(num_of_robots)]
        names = [line.split()[1] for line in lines]
    
    print("\033[36m \nCreating temporary launch file.\033[0m")    
    create_launch_file(launch_file, names)

    cli_args = [launch_file]

    print("\033[36mStarting Spheros.\033[0m")
    # Prepare for launching.
    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    roslaunch.configure_logging(uuid)
    roslaunch_args = cli_args[1:]
    roslaunch_file = [(roslaunch.rlutil.resolve_launch_arguments(cli_args)[0], roslaunch_args)]

    # Launch!
    launch = roslaunch.parent.ROSLaunchParent(uuid, roslaunch_file)
    launch.start()

    print("\033[92mSpheros started. Press Ctrl-C to exit when done. \033[0m")

    # Keep it from exiting.
    try:
        launch.spin()
    finally:
        # After Ctrl+C, stop all nodes from running.
        launch.shutdown()
        # Comment out this line if you want to keep the file.
        os.remove(launch_file)
    

if __name__ == '__main__':
    rospy.init_node('stage_launcher', anonymous=True)

    print("\033[36m \nInitialized launcher node. Starting launch process. \033[0m")

    main()





