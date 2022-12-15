# sphero_driver_v2

This is a new version of the Python ROS driver for Sphero robots. It essentially wraps the functionality of [spherov2.py](https://github.com/artificial-intelligence-class/spherov2.py) Python API for use in ROS. For the old driver, check out [sphero_sprk_ros](https://github.com/antonellabarisic/sphero_sprk_ros).

**Difference between drivers:** The new driver theoretically supports all Sphero toys, while the old one is intended only for Sphero SPRK+. The new driver also enables reading from the sensor, something that is currently broken in the ROS Noetic version of the old driver.

## <a name="Usage"></a> Usage

First, edit the order of Spheros in the [config file](cfg/sphero_addresses.txt). When starting the driver, the first `n` Sphero names from the file will be used to establish a connection. The first Sphero in the list will get namespace `sphero_0`, the second one `sphero_1`, and so on.

Start the ROS core and then run the driver nodes:
```
rosrun sphero_driver_v2 drivers.launch.py n
```

## <a name="pckg"></a> Driver description

### <a name="sub"></a> Subscribed topics
- ```/sphero_0/cmd_vel```
  - Type: geometry_msgs/Twist
- ```/sphero_0/set_color```
  - Type: std_msgs/ColorRGBA
- ```/sphero_0/set_heading```
  - Type: std_msgs/Float32
- ```/sphero_0/set_angular_velocity```
  - Type: std_msgs/Float32
- ```/sphero_0/disable_stabilization```
  - Type: std_msgs/Bool
- ```/sphero_0/manual_calibration```
  - Type: std_msgs/Bool

### <a name="pub"></a> Published topics
- ```/sphero_0/imu```
  - Type: sensor_msgs/Imu
- ```/sphero_0/odom```
  - Type: nav_msgs/Odometry
- ```/sphero_0/battery```
  - Type: diagnostic_msgs/DiagnosticArray