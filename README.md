# 4DRadarSLAM
## A 4D Imaging Radar SLAM System for Large-scale Environments based on Pose Graph Optimization
***4dRadarSLAM*** is an open source ROS package for real-time 6DOF SLAM using a 4D Radar. It is based on 3D Graph SLAM with Uncertainty GICP scan matching-based odometry estimation and Intensity Scan Context loop detection. It also supports several graph constraints, such as GPS. We have tested this package with ***Oculli Eagle*** in outdoor structured (buildings), unstructured (trees and grasses) and semi-structured environments.

## 1. Dependency
### 1.1 **Ubuntu** and **ROS**
Ubuntu 64-bit 16.04, 18.04 or 20.04.
ROS Kinetic, Melodic or Noetic. [ROS Installation](http://wiki.ros.org/ROS/Installation):

### 1.2 ***4DRadarSLAM*** requires the following libraries:

- Eigen3
- OpenMP
- PCL
- g2o
### 1.3 The following ROS packages are required:
- geodesy
- nmea_msgs
- pcl_ros
- Our modified [fast_ugicp](https://github.com/zhuge2333/fast_ugicp), in which Uncertainty GICP algorithum module is added. The original is [fast_gicp](https://github.com/SMRT-AIST/fast_gicp)
```
    sudo apt-get install ros-XXX-geodesy ros-XXX-pcl-ros ros-XXX-nmea-msgs ros-XXX-libg2o
```
**NOTICE:** remember to replace "XXX" on above command as your ROS distributions, for example, if your use ROS-noetic, the command should be:
```
    sudo apt-get install ros-noetic-geodesy ros-noetic-pcl-ros ros-noetic-nmea-msgs ros-noetic-libg2o
```

## 2. System architecture
***4DRadarSLAM*** consists of three nodelets.

- *preprocessing_nodelet*
- *scan_matching_odometry_nodelet*
- *radar_graph_slam_nodelet*

The input point cloud is first downsampled by ***preprocessing_nodelet***, estimate its ego velocity and remove dynamic objects, and then passed to the next nodelets. While scan_matching_odometry_nodelet estimates the sensor pose by iteratively applying a scan matching between consecutive frames (i.e., odometry estimation). The estimated odometry are sent to ***radar_graph_slam***. To compensate the accumulated error of the scan matching, it performs loop detection and optimizes a pose graph which takes various constraints into account.

<div align="center">
    <img src="doc/fig_flowchart_4dradarslam.png" width = 60% >
</div>

## 3. Parameter tuning guide
.

## 4. Run the package
Download [Our recorded rosbag]() and then
```
roslaunch radar_graph_slam radar_graph_slam.launch
```
You'll see a point cloud like:
<div align="center">
    <img src="doc/fig_qualitative_map.png" width = 60% >
</div>

## 5. Evaluate the results
In our paper, we use [rpg_trajectory_evaluation](https://github.com/uzh-rpg/rpg_trajectory_evaluation.git), the performance indices used are RE (relative error) and ATE (absolute trajectory error).

## 6. Collect your own datasets
You need to calibrate the transform between Radar and Lidar

## 7. Acknowlegement
4DRadarSLAM is based on hdl_graph_slam (Kenji Koide, Jun Miura, and Emanuele Menegatti, A Portable 3D LIDAR-based System for Long-term and Wide-area People Behavior Measurement, Advanced Robotic Systems, 2019).
