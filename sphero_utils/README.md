# sphero_utils

Various utility things not suitable for other packages.

### List of available utilities
- `nodes/marker_server.py` - A centralized node that can subscribe to different topics and publish Rviz markers to visualize them. Currently, it can be used to visualize estimated velocities from the Kalman filter.
- `rviz/kalman.rviz` - Rviz configuration file for visualizing Kalman velocity markers and Sphero TFs.