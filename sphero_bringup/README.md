# sphero_bringup

A single starting place for launching all of the desired functionalities.
First, start the Sphero driver and then one of the following functionalities.

### List of currently available launch files:
- `only_joystick.launch`: Manual control with Logitech F710 joystick.
- `odometry_pid.launch`: Manual control with Logitech F710 joystick (default disabled) and position PID controller.
- `optitrack_joystick.launch`: Manual control with Logitech F710 joystick and OptiTrack tracking.
- `optitrack_pid.launch`: Manual control with Logitech F710 joystick (default disabled), OptiTrack tracking, and position PID controller.
- `webcam_joystick.launch`: Manual control with Logitech F710 joystick and webcam tracking.
- `webcam_pid.launch`: Manual control with Logitech F710 joystick (default disabled), webcam tracking, and position PID controller.
- `webcam_ext.launch`: Manual control with Logitech F710 joystick (default disabled), webcam tracking, and external controller. This is a template file for adding custom external controllers.

All launch files take the parameters from `params/default.yaml` and load them on ROS parameter server for all nodes to use.


### Notes on joystick
All launch files that include the joystick have the following line to start the appropriate Linux driver:
```xml
  <node pkg="joy" type="joy_node" name="JoyInput">
      <param name="dev" type="string" value="$(arg joy_path)"/>
      <param name="deadzone" value="0"/>
  </node>
```
Argument `joy_path` is set at the top of the file as
```xml
<arg name="joy_path" default="/dev/input/js0"/> 	<!-- Joystick device path. -->
```
In some cases, your path may be different than the default one. To check that, before plugging in the joystick, take a look at the output of the command `ls /dev/input`. If there is already `js0` entry in the output, you will not be able to use the default launch file. Now plug in the joystick and look at the output of `ls /dev/input` once more. A new entry should appear.  

Change the default path in **all** launch files you are using, for example:
```xml
<arg name="joy_path" default="/dev/input/js1"/> 	<!-- Joystick device path. -->
```

Alternatively, you can specify the joystick path in the command line before launching:
```bash
roslaunch sphero_bringup only_joystick.launch joy_path:=/dev/input/js1
```
In this case, you don't have to change anything in the launch files.

### Twist multiplexor
All of these launch files utilize a twist command multiplexor. This node subscribes to multiple sources of commanded velocity and outputs `cmd_vel` based on priorities and triggers. The joystick (`man_vel` topic) is configured to have the highest priority so it overwrites all other sources. When the joystick is disabled, other commands can go through (such as `pid_vel` and `ext_vel`).

The advantage of such an approach is that you can easily take over a non-responding controller or one sending out erroneous data.

