# sphero_control

A package for various Sphero controllers.

### List of currently available controllers:
- Position-based PID controller (`pid.py` and `position_pid_node.py`)

### Provided topics:
- Publishers:
    - `/<robot_name>/cmd_vel` (geometry_msgs/Twist) - Commanded velocity (in 0-255 range)
- Subscribers:
    - `/<robot_name>/odom` (nav_msgs/Odometry) - Robot's odometry (real or estimated). Can be remapped as required.
    - `/<robot_name>/reference` (geometry_msgs/Point) - 2D reference point.
