# sphero_bringup

A single starting place for launching all of the desired functionalities.

### List of currently available launch files:
- `only_joystick.launch`: Manual control with Logitech F710 joystick
- `optitrack_joystick.launch`: Manual control with Logitech F710 joystick and OptiTrack tracking
- `optritrack_pid.launch`: Manual control with Logitech F710 joystick (default disabled), OptiTrack tracking and position PID controller

All launch files take the parameters from `params/default.yaml` and load them on ROS parameter server for all nodes to use.
