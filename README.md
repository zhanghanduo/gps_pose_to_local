# GPS Conversion

## Description
This is a package based on ROS tested on ROS Kinetics, Ubuntu 16.04.

GPS pose information (`geometry_msgs::PoseWithCovarianceStamped`) is the input.

## Dependency

Eigen3


## Road Map
- [x] Read external pose information
- [x] Convert to camera frame pose
- [x] Publish new camera frame pose topic
- [x] Publish corresponding TF