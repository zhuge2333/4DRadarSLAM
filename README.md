# 4DRadarSLAM
## A 4D Imaging Radar SLAM System for Large-scale Environments based on Pose Graph Optimization
***4dRadarSLAM*** is an open source ROS package for real-time 6DOF SLAM using a 4D Radar. It is based on 3D Graph SLAM with Uncertainty GICP scan matching-based odometry estimation and Intensity Scan Context loop detection. It also supports several graph constraints, such as GPS. We have tested this package with ***Oculli Eagle*** in outdoor structured (buildings), unstructured (trees and grasses) and semi-structured environments.

## 1. Dependency
### 1.1 **Ubuntu** and **ROS**
Ubuntu 64-bit 16.04, 18.04 or 20.04.
ROS Kinetic, Melodic or Noetic. [ROS Installation](http://wiki.ros.org/ROS/Installation):

### 1.2 ***4DRadarSLAM*** requires the following libraries:

- OpenMP
- PCL
- g2o
- suitesparse

### 1.3 The following ROS packages are required:
- geodesy
- nmea_msgs
- pcl_ros
- [fast_gicp](https://github.com/SMRT-AIST/fast_gicp)
```
    sudo apt-get install ros-XXX-geodesy ros-XXX-pcl-ros ros-XXX-nmea-msgs ros-XXX-libg2o
```
**NOTICE:** remember to replace "XXX" on above command as your ROS distributions, for example, if your use ROS-noetic, the command should be:

```
    sudo apt-get install ros-noetic-geodesy ros-noetic-pcl-ros ros-noetic-nmea-msgs ros-noetic-libg2o
```

## 2. System architecture


## 3. Parameter tuning guide


## 4. Run the package
Download [Our recorded rosbag]() and then
```
roslaunch radar_graph_slam radar_graph_slam.launch
```
You'll see a point cloud like:
<div align="center">
    <img src="pics/indoor_aggressive_bag_res.png" width = 60% >
</div>

and .
<div align="center">
    <img src="pics/cover.png" width = 60% >
</div>

## 5. Collect your own datasets
You need to calibrate the transform between Radar and Lidar

## 6. License
The source code is released under [GPLv3](http://www.gnu.org/licenses/) license.

We are still working on improving the performance and reliability of our codes. For any technical issues, please contact me via email  <  >.

For commercial use, please contact Dr.  <  >
