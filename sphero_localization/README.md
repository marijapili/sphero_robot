# sphero_localization

A package for external localization of Sphero robots.

### List of currently available localization methods:
- OptiTrack
    - A list of (unsorted and unidentified) positions is streamed via [mocap_optitrack](https://github.com/larics/mocap_optitrack).
    - Kalman filter is used to identify Spheros and estimate velocity.
    - Initial positioning and identification is accomplished with `sphero_init.py` script which sequentially turns on Sphero's LEDs and stores the only available position from mocap_optitrack.
- Kalman filter
    - Can be used with other localization methods trough simple topic remapping.
    - If position of each Sphero is exactly known (one-on-one mapping), set the `/data_associated` parameter to true.
    

### Provided topics:
- Publishers:
    - `/<robot_name>/odom_est` (nav_msgs/Odometry) - Estimated position and velocity
    - `/<robot_name>/debug_est` (nav_msgs/Odometry) - Estimated position and velocity at higher frequency
- Subscribers:
    - `/<robot_name>/odom` (nav_msgs/Odometry or geometry_msgs/PoseArray) - Position streamed from the external sensor. <br>
    (_Note: Kalman filter currently expects inputs only from OptiTrack system or stage_ros simulator. The former uses PoseArray messages and position data is not mapped. The latter uses Odometry and the streamed positions are unique for each robot. This will be changed in the future to accept a wider range of measurements._)
