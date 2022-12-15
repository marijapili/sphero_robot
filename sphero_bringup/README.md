# sphero_bringup

A single starting place for launching all of the desired functionalities.
First start the Sphero driver and then one of the following functionalities.

### List of currently available launch files:
- `only_joystick.launch`: Manual control with Logitech F710 joystick.
- `odometry_pid.launch`: Manual control with Logitech F710 joystick (default disabled) and position PID controller.
- `optitrack_joystick.launch`: Manual control with Logitech F710 joystick and OptiTrack tracking.
- `optitrack_pid.launch`: Manual control with Logitech F710 joystick (default disabled), OptiTrack tracking and position PID controller.
- `webcam_joystick.launch`: Manual control with Logitech F710 joystick and webcam tracking.
- `webcam_pid.launch`: Manual control with Logitech F710 joystick (default disabled), webcam tracking and position PID controller.

All launch files take the parameters from `params/default.yaml` and load them on ROS parameter server for all nodes to use.
