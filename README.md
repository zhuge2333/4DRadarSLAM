# 4DRadarSLAM
## A 4D Imaging Radar SLAM System for Large-scale Environments based on Pose Graph Optimization

[Paper (ResearchGate)](https://www.researchgate.net/publication/371309896_4DRadarSLAM_A_4D_Imaging_Radar_SLAM_System_for_Large-scale_Environments_based_on_Pose_Graph_Optimization), [IEEEXplore](https://ieeexplore.ieee.org/document/10160670), [Video](https://www.youtube.com/watch?v=Qlvs7ywA5TI), [Dataset (NTU4DRadLM)](https://github.com/junzhang2016/NTU4DRadLM)

***4DRadarSLAM*** is an open source ROS package for real-time 6DOF SLAM using a 4D Radar. It is based on 3D Graph SLAM with Adaptive Probability Distribution GICP scan matching-based odometry estimation and Intensity Scan Context loop detection. It also supports several graph constraints, such as GPS. We have tested this package with ***Oculli Eagle*** in outdoor structured (buildings), unstructured (trees and grasses) and semi-structured environments.

4DRadarSLAM can operate in adverse wheather. We did a experiment in which sensors are covered by dense ***Smoke***. The Lidar SLAM (R2LIVE) failed, but our 4DRadarSLAM is not affected by it, thanks to the penetration of millimeter waves to small objects such as smoke and rain.

<p align='center'>
    <img src="./doc/mapping_smoke.gif" alt="drawing" width="800"/>
</p>


## 1. Dependency
### 1.1 **Ubuntu** and **ROS**
Ubuntu 64-bit 18.04 or 20.04.
ROS Melodic or Noetic. [ROS Installation](http://wiki.ros.org/ROS/Installation):

### 1.2 ***4DRadarSLAM*** requires the following libraries:
- Eigen3
- OpenMP
- PCL
- g2o
### 1.3 The following ROS packages are required:
- geodesy
- nmea_msgs
- pcl_ros
- Our modified [fast_apdgicp](https://github.com/zhuge2333/fast_apdgicp), in which Adaptive Probability Distribution GICP algorithum module is added. The original is [fast_gicp](https://github.com/SMRT-AIST/fast_gicp)
- [barometer_bmp388](https://github.com/zhuge2333/barometer_bmp388.git)
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

The input point cloud is first downsampled by ***preprocessing_nodelet***; the radar pointcloud is transformed to Livox LiDAR frame; estimate its ego velocity and remove dynamic objects, and then passed to the next nodelets. While scan_matching_odometry_nodelet estimates the sensor pose by iteratively applying a scan matching between consecutive frames (i.e., odometry estimation). The estimated odometry are sent to ***radar_graph_slam***. To compensate the accumulated error of the scan matching, it performs loop detection and optimizes a pose graph which takes various constraints into account.

<div align="center">
    <img src="doc/fig_flowchart_system.png" width = 100% >
</div>

## 3. Parameter tuning guide
The mapping quality largely depends on the parameter setting. In particular, scan matching parameters have a big impact on the result. Tune the parameters accoding to the following instructions:

### 3.1 Point cloud registration
- ***registration_method***

This parameter allows to change the registration method to be used for odometry estimation and loop detection. Our code gives five options: ICP, NDT_OMP, FAST_GICP, FAST_APDGICP, FAST_VGICP. 

FAST_APDGICP is the implementation of our proposed Adaptive Probability Distribution GICP, it utilizes OpenMP for acceleration. Note that FAST_APDGICP requires extra parameters.
Point uncertainty parameters:
- ***dist_var***
- ***azimuth_var***
- ***elevation_var***

*dist_var* means the uncertainty of a point’s range measurement at 100m range, *azimuth_var* and *elevation_var* denote the azimuth and elevation angle accuracy (degree)

### 3.2 Loop detection 
- ***accum_distance_thresh***: Minimum distance beteen two edges of the loop
- ***min_loop_interval_dist***: Minimum distance between a new loop edge and the last one
- ***max_baro_difference***: Maximum altitude difference beteen two edges' odometry
- ***max_yaw_difference***: Maximum yaw difference beteen two edges' odometry
- ***odom_check_trans_thresh***: Translation threshold of Odometry Check
- ***odom_check_rot_thresh***: Rotation threshold of Odometry Check
- ***sc_dist_thresh***: Matching score threshold of Scan Context

### 3.3 Other parameters
  All the configurable parameters are available in the launch file. Many are similar to the project ***hdl_graph_slam***.

## 4. Run the package
Download [our recorded rosbag](https://drive.google.com/drive/folders/14jVa_dzmckVMDdfELmY32fJlKrZG1Afv?usp=sharing)  (**More datasets**: [NTU4DRadLM](https://github.com/junzhang2016/NTU4DRadLM)) and, then
```
roslaunch radar_graph_slam radar_graph_slam.launch
```
You'll see a point cloud like:
<div align="center">
    <img src="doc/fig_carpark_map.png" width = 80% >
</div>
You can choose the dataset to play at end of the launch file.
In our paper, we did evaluation on five datasets, mapping results are presented below:
<div align="center">
    <img src="doc/fig_map_compare.png" width = 100% >
</div>



## 5. Evaluate the results
In our paper, we use [rpg_trajectory_evaluation](https://github.com/uzh-rpg/rpg_trajectory_evaluation.git), the performance indices used are RE (relative error) and ATE (absolute trajectory error).

## 6. Collect your own datasets
You need a 4D Imaging radar (we use Oculii's Eagle). Also, a barometer (we use BMP388) and GPS/RTK-GPS (we use ZED-F9P) are optional. If you need to compare Lidar SLAM between the algorithum, or use its trajectory as ground truth, calibrating the transform between Radar and Lidar is a precondition.

## 7. Acknowlegement
1. 4DRadarSLAM is based on [koide3/hdl_graph_slam](https://github.com/koide3/hdl_graph_slam) 
2. [irapkaist/scancontext](https://github.com/irapkaist/scancontext) scan context
3. [wh200720041/iscloam](https://github.com/wh200720041/iscloam) intensity scan context
4. [christopherdoer/reve](https://github.com/christopherdoer/reve) radar ego-velocity estimator
5. [NeBula-Autonomy/LAMP](https://github.com/NeBula-Autonomy/LAMP) odometry check for loop closure validation
6. [slambook-en](https://github.com/gaoxiang12/slambook-en) and [Dr. Gao Xiang (高翔)](https://github.com/gaoxiang12). His SLAM tutorial and blogs are the starting point of our SLAM journey.
7. [lvt2calib](https://github.com/Clothooo/lvt2calib) by LIUY Clothooo for sensor calibration.
## 8. Citation
If you find this work is useful for your research, please consider citing:
```
@INPROCEEDINGS{ZhangZhuge2023ICRA,
  author={Zhang, Jun and Zhuge, Huayang and Wu, Zhenyu and Peng, Guohao and Wen, Mingxing and Liu, Yiyao and Wang, Danwei},
  booktitle={2023 IEEE International Conference on Robotics and Automation (ICRA)}, 
  title={4DRadarSLAM: A 4D Imaging Radar SLAM System for Large-scale Environments based on Pose Graph Optimization}, 
  year={2023},
  volume={},
  number={},
  pages={8333-8340},
  doi={10.1109/ICRA48891.2023.10160670}}
```


```
@INPROCEEDINGS{ZhangZhugeLiu2023ITSC,  
author={Jun Zhang∗, Huayang Zhuge∗, Yiyao Liu∗, Guohao Peng, Zhenyu Wu, Haoyuan Zhang, Qiyang Lyu, Heshan Li, Chunyang Zhao, Dogan Kircali, Sanat Mharolkar, Xun Yang, Su Yi, Yuanzhe Wang+ and Danwei Wang},  
booktitle={2023 IEEE 26th International Conference on Intelligent Transportation Systems (ITSC)},   
title={NTU4DRadLM: 4D Radar-centric Multi-Modal Dataset for Localization and Mapping},  
year={2023},  
volume={},  
number={},  
pages={},  
doi={}}
```
