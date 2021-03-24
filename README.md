# ROS packages for Sphero robots.

Various ROS packages for Sphero robots, usable in simulation and with real robots, including localization, control, teleoperation and more. The driver was developed specifically for Sphero SPRK+ model, but with an appropriate driver, these packages can be used for other robots as well.

This repository currently consists of following packages:
- `sphero_bringup`: Launch files for starting common functionallity groups
- `sphero_control`: Automatic control of Sphero robots (PID controllers, velocity trackers, ...)
- `sphero_localization`: External localization and state estimation (Kalman filter, visual positioning, ...)
- `sphero_teleop`: Manual control of Sphero robots (joystick, keyboard, ...)
- `sphero_sprk_ros`: ROS/Python driver for Sphero SPRK+.

For detailed information about each package, please look at their respective READMEs.

More packages are coming soon:
- `sphero_simulation`: Gazebo simulation
- `sphero_description`: urdf and visual files for Sphero robots
- ... 

### Requirements
- Ubuntu 16.04 or Ubuntu 18.04
- ROS Kinetic or ROS Melodic
    - Required packages: `ros-<version>-joy`
- Python 2.7
    - Additional libraries: `numpy`
    
### Installation
1. Install all the requirements.
1. Create a new catkin workspace or use an existing one.
1. Clone the repository in the `src` folder of your workspace and update git submodules:
    ```shell script
    $ cd <path_to_your_ws>/src/
    $ git clone --recursive git@github.com:larics/sphero_robot.git
    ```
1. Build everything using catkin_tools (recommended):
    ```shell script
    $ catkin build
    ```
   or catkin_make:
   ```shell script
   $ cd <path_to_your_ws>
   $ catkin_make
   ```
1. If you want to get the latest changes, use:
    ```shell script
    $ git pull --recurse-submodules
    ```

### Usage
It is recommended to launch the drivers for the selected number of Spheros in a seperate terminal using the launch file provided in sphero_sprk_ros package (sometimes it is necessary to re-start the drivers a few times before all connections are sucessful). In a second terminal, use one of the provided launch files in sphero_bringup package.
```shell script
$ roslaunch sphero_sprk_ros drivers.launch  # 1st terminal
$ roslaunch sphero_bringup <desired_functionallity>.launch  # 2nd terminal
```
Otherwise, feel free to look into the packages and write your own launch files if the desired combination is not available.
