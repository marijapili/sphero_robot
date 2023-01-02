# sphero_localization

A package for external localization of Sphero robots.

### List of currently available localization methods:
- OptiTrack
    - A list of (unsorted and unidentified) positions is streamed via [mocap_optitrack](https://github.com/larics/mocap_optitrack).
    - Kalman filter is used to identify Spheros and estimate velocity.
    - Initial positioning and identification is accomplished with `sphero_init.py` script which sequentially turns on Sphero's LEDs and stores the only available position from mocap_optitrack.
    - **Above functionality is currently broken and will be fixed to match webcam localization in the future.**
- Webcam
    - Uses a color or brightness filter (HSV values in general) to separate the Spheros from the background.
    - Using known camera properties, the centers of detected Spheros are transformed to the world coordinate frame.
    - Currently, SORT is used to associate measurements to Spheros and track them throughout the execution.
- SORT
    - Uses its own implementation of Kalman filter to track individual detected objects.
    - In each step, new detections are matched with predictions from the last step so that the overall distance between them is minimized.
- Kalman filter
    - Used for providing a stable stream of position in case of sensor dropouts and for estimating the velocity.
    - Can be used with other localization methods trough simple topic remapping.
    - One Kalman filter node is used for each Sphero in its namespace. That means the measurements must be associated with correct Spheros.

### Package structure
- **nodes** - ROS nodes.
    - `kalman_filter_node.py` - One node for each Sphero.
    - `webcam_tracking.py` - A centralized node for streaming the image, processing, detection, tracking, and publishing positions.
- **scripts** - Executable Python files that are intended to run independently from ROS.
    - `camera_calibration` - Scripts for camera calibration. Run once for a new camera setup.
    - `camer_stitching` - Scripts for stitching the feeds from two cameras.
    - `filter_param_finder.py` - An interactive way to determine the optimal HSV or RGB filter parameters.
    - `tracking_test.py` - Testing the webcam tracking without ROS.
- **src** - Python files imported in nodes and scripts.
    - `blob_detector.py` - Pipeline for detecting blobs in image.
    - `duo_c270.py` - Pipeline for grabbing and processing frames from two Logitech C270 cameras. Can be used by other scripts.
    - `kalman_filter.py` - Custom implementation of Kalman filter.
    - `sort_wrapper.py` - ROS wrapper for SORT tracking. Currently not in use.
    - `sort.py` - Implementation of SORT tracking algorithm.
    - `sphero_init.py` - Procedure for getting initial Sphero positions. Currently not in use.

### Detailed explanation of webcam localization
1. We are using the [config file](config/config.yaml) to define which camera ports are used and where is their calibration data.
1. `webcam_tracking.py` initializes the `FrameServer`, `SpheroBlobDetector`, and `Sort` and then runs the loop.
1. `FrameServer` in `duo_c270.py` grabs the frame(s) from one or two cameras, undistorts them using provided calibration data, and optionally stitches two feeds together. It is also used to transform image coordinates to the world frame.
1. In the `SpheroBlobDetector` from `blob_detector.py`, the image frame from the `FrameServer` is blurred, filtered, and masked to highlight the pixels corresponding to the Spheros. Individual blobs are then located and their centers and sizes are fed to the `Sort` tracker in `sort.py`.
1. `Sort` returns the list of bounding boxes of detected objects and their IDs.
1. Using the object bounding boxes and IDs, `webcam_tracking.py` node publishes Sphero positions on their individual topics.
1. `kalman_filter_node.py` subscribes to position topic and estimates the velocity.

### Provided topics:
- Publishers:
    - `/<robot_name>/odom_est` (nav_msgs/Odometry) - Estimated position and velocity.
- Subscribers:
    - `/<robot_name>/position` (geometry_msgs/Point) - Position streamed from the external sensor.

### Map server
Map server is not included in any of the launch files because using maps is not a default use case of this package.
However, several maps are provided for Multi-Robot Systems 2022/2023 course for convenience and here is an example 
of how to include the map server in your launch files.
```xml
<arg name="map_name" default="empty"/>
<arg name="map_yaml" default="$(find sphero_localization)/maps/$(arg map_name)/$(arg map_name).yaml"/>
<node pkg="map_server" type="map_server" args="$(arg map_yaml)" name="map_server">
    <param name="frame_id" value="world"/>
</node>
```


### TODO:
- [ ] Separate ROS from non-ROS code