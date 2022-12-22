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


### Twist multiplexor
All of these launch files utilize a twist command multiplexor. This node subscribes to multiple sources of commanded velocity and outputs `cmd_vel` based on priorities and triggers. The joystick (`man_vel` topic) is configured to have the highest priority so it overwrites all other sources. When the joystick is disabled, other commands can go through (such as `pid_vel` and `ext_vel`).

The advantage of such an approach is that you can easily take over a non-responding controller or one sending out erroneous data.

